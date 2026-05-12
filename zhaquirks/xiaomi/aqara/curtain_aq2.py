"""Aqara Roller Shade Controller (lumi.curtain.aq2) device."""

from __future__ import annotations

from typing import Any

from zigpy import types as t
from zigpy.quirks.v2 import QuirkBuilder
from zigpy.zcl import AttributeReadEvent, AttributeReportedEvent, foundation
from zigpy.zcl.clusters.closures import WindowCovering
from zigpy.zcl.clusters.general import AnalogOutput

from zhaquirks import CustomCluster
from zhaquirks.xiaomi import LUMI, BasicCluster


class AnalogOutputCurtainAq2(CustomCluster, AnalogOutput):
    """AnalogOutput cluster reporting current position and used for writing target position."""

    _CONSTANT_ATTRIBUTES = {
        AnalogOutput.AttributeDefs.description.id: "Current position",
        AnalogOutput.AttributeDefs.max_present_value.id: 100.0,
        AnalogOutput.AttributeDefs.min_present_value.id: 0.0,
        AnalogOutput.AttributeDefs.out_of_service.id: 0,
        AnalogOutput.AttributeDefs.resolution.id: 1.0,
        AnalogOutput.AttributeDefs.status_flags.id: 0x00,
    }

    def __init__(self, *args, **kwargs):
        """Init."""
        super().__init__(*args, **kwargs)
        self.on_event(AttributeReadEvent.event_type, self._handle_attribute_read)
        self.on_event(
            AttributeReportedEvent.event_type, self._handle_attribute_reported
        )

    def _handle_attribute_read(self, event: AttributeReadEvent) -> None:
        """Handle attribute read events — update WindowCovering position."""
        if event.attribute_id == self.AttributeDefs.present_value.id:
            self.endpoint.window_covering.update_attribute(
                WindowCovering.AttributeDefs.current_position_lift_percentage.id,
                t.uint8_t(100 - event.value),
            )

    def _handle_attribute_reported(self, event: AttributeReportedEvent) -> None:
        """Handle device attribute reports.

        The device sends exactly two position reports per commanded movement:
          1. An echo of the actual position ~30 ms after the write ACK (the motor
             has already begun moving — e.g. starting at 100% open and commanding
             0% will produce an echo around 97%).
          2. A final-position report when the curtain arrives at the target.

        After a stop or a physically-driven movement the device sends a single
        natural position report ~1 s later with the actual stopped position.

        Behaviour, distinguished by WindowCovering's _is_moving flag:

        - was_moving=True  → echo report: clear the flag and return without
          updating WindowCovering.  The echo's position is accurate, but pushing
          it through to ZHA would call _start_lift_transition(is_position_update=
          True), which caps the cover entity's transition timer at ZHA's
          DEFAULT_MOVEMENT_TIMEOUT (5 s).  Travel time for this device routinely
          exceeds 5 s, so the timer would expire mid-movement and ZHA would
          prematurely transition the entity to OPEN/CLOSED.  Discarding the echo
          preserves the initial long timer (sized by distance to target), keeping
          the cover in OPENING/CLOSING for the entire travel.

        - was_moving=False → final, post-stop, or unsolicited report.  Drive
          WindowCovering twice with the same value in the same event-loop turn.
          When ZHA's _target_lift_position is None (post-stop or unsolicited),
          _determine_state would otherwise infer a target from the previous→
          current direction (e.g. 100 → 73 → infer target=CLOSED) and mark the
          cover as CLOSING/OPENING despite the device having stopped — then
          flip to OPEN/CLOSED 5 s later when the transition timer expires.
          A second update at the same position leaves _lift_position_history
          at [v, v] so the direction-inference branch is skipped (it requires
          previous != current) and the state resolves immediately to OPEN/
          CLOSED.  Harmless when a target is set (current == target already
          resolves directly).
        """
        if event.attribute_id == self.AttributeDefs.present_value.id:
            wc = self.endpoint.window_covering
            was_moving = wc._is_moving
            wc._is_moving = False
            if was_moving:
                return
            wc_attr = WindowCovering.AttributeDefs.current_position_lift_percentage.id
            wc_pos = t.uint8_t(100 - event.value)
            wc.update_attribute(wc_attr, wc_pos)
            wc.update_attribute(wc_attr, wc_pos)


class WindowCoveringCurtainAq2(CustomCluster, WindowCovering):
    """Window covering cluster for handling motor commands."""

    _CONSTANT_ATTRIBUTES = {
        WindowCovering.AttributeDefs.window_covering_type.id: WindowCovering.WindowCoveringType.Rollershade,
    }

    def __init__(self, *args, **kwargs):
        """Init."""
        super().__init__(*args, **kwargs)
        # Tracks whether a commanded movement is in progress.  Set by command()
        # when issuing up_open/down_close/go_to_lift_percentage and cleared
        # either by the echo handler (AO._handle_attribute_reported) or by a
        # successful stop.  Read in two places:
        #  - AO._handle_attribute_reported uses it to distinguish the echo
        #    report from the final/post-stop report.
        #  - read_attributes uses it to suppress the post-command position
        #    poll, which would otherwise trigger ZHA's transition-timer cap
        #    (see read_attributes docstring).
        self._is_moving = False

    async def command(
        self,
        command_id: foundation.GeneralCommand | int | t.uint8_t,
        *args: Any,
        manufacturer: int | t.uint16_t | None = None,
        expect_reply: bool = True,
        tsn: int | t.uint8_t | None = None,
        **kwargs: Any,
    ) -> Any:
        """Route commands to AnalogOutput writes; stop uses standard ZCL stop command.

        Movement commands (up_open, down_close, go_to_lift_percentage) write the
        target position to AnalogOutput present_value and set _is_moving=True.

        The device responds with two attribute reports per movement:
          1. An echo report (~30 ms after the write ACK) with the position the
             motor has already reached.
          2. A final-position report when the curtain arrives.
        The echo clears _is_moving and is discarded; the final report drives
        two synchronous WindowCovering updates.  See
        AnalogOutputCurtainAq2._handle_attribute_reported for the rationale.

        No explicit position read is issued after a movement or stop command.
        Pushing a position update mid-movement would trigger ZHA's
        _start_lift_transition(is_position_update=True), capping the cover
        entity's transition timer at DEFAULT_MOVEMENT_TIMEOUT (5 s) — shorter
        than this device's travel time — which would prematurely transition
        the entity to OPEN/CLOSED before the curtain arrives.  _is_moving
        suppresses the post-command poll inside read_attributes for the same
        reason.
        """
        if command_id == WindowCovering.ServerCommandDefs.up_open.id:
            self._is_moving = True
            (res,) = await self.endpoint.analog_output.write_attributes(
                {AnalogOutput.AttributeDefs.present_value.name: 100.0}
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=res[0].status)

        if command_id == WindowCovering.ServerCommandDefs.down_close.id:
            self._is_moving = True
            (res,) = await self.endpoint.analog_output.write_attributes(
                {AnalogOutput.AttributeDefs.present_value.name: 0.0}
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=res[0].status)

        if command_id == WindowCovering.ServerCommandDefs.go_to_lift_percentage.id:
            self._is_moving = True
            (res,) = await self.endpoint.analog_output.write_attributes(
                {AnalogOutput.AttributeDefs.present_value.name: float(100 - args[0])}
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=res[0].status)

        if command_id == WindowCovering.ServerCommandDefs.stop.id:
            result = await super().command(
                command_id,
                *args,
                manufacturer=manufacturer,
                expect_reply=expect_reply,
                tsn=tsn,
                **kwargs,
            )
            # Only clear once the stop has been acknowledged: if the send fails, the
            # curtain is still moving and position reads should remain suppressed.
            self._is_moving = False
            # The device sends a natural position report ~1 s after stopping; that
            # report drives the WindowCovering update via _handle_attribute_reported.
            return result

        return foundation.GENERAL_COMMANDS[
            foundation.GeneralCommand.Default_Response
        ].schema(command_id=command_id, status=foundation.Status.UNSUP_CLUSTER_COMMAND)

    async def read_attributes(
        self,
        attributes: list[int | str | foundation.ZCLAttributeDef],
        **kwargs,
    ) -> Any:
        """Redirect current_position_lift_percentage reads to AnalogOutput present_value.

        While _is_moving is True the redirect is suppressed.  Letting a read
        complete mid-movement would push a position update through to ZHA,
        triggering _start_lift_transition(is_position_update=True) and capping
        the cover entity's transition timer at DEFAULT_MOVEMENT_TIMEOUT (5 s) —
        shorter than this device's travel time, causing a premature OPEN/CLOSED
        transition.  The last cached position is returned instead so the caller
        always gets a defined response for the attribute.
        """
        success = {}
        failure = {}

        pos_attr_def = WindowCovering.AttributeDefs.current_position_lift_percentage
        pos_attr_key = next(
            (attr for attr in attributes if self.find_attribute(attr) == pos_attr_def),
            None,
        )

        if pos_attr_key is not None:
            remaining = [a for a in attributes if a != pos_attr_key]
            if not self._is_moving:
                pv = AnalogOutput.AttributeDefs.present_value
                (
                    ao_success,
                    ao_failure,
                ) = await self.endpoint.analog_output.read_attributes([pv], **kwargs)
                if pv in ao_success:
                    success[pos_attr_key] = t.uint8_t(100 - ao_success[pv])
                if pv in ao_failure:
                    failure[pos_attr_key] = ao_failure[pv]
            else:
                # Return the last cached position rather than reading the device.
                # A live read would propagate a position update to ZHA mid-
                # movement, capping its transition timer at 5 s (see docstring).
                # If nothing is cached yet, surface a defined failure rather
                # than silently dropping the attribute from the response.
                cached = self._attr_cache.get(pos_attr_def.id)
                if cached is not None:
                    success[pos_attr_key] = cached
                else:
                    failure[pos_attr_key] = foundation.Status.FAILURE
        else:
            remaining = attributes

        # Read the remaining ones directly
        other_success, other_failure = await super().read_attributes(
            remaining, **kwargs
        )
        success.update(other_success)
        failure.update(other_failure)

        return success, failure


(
    QuirkBuilder(LUMI, "lumi.curtain.aq2")
    .prevent_default_entity_creation(endpoint_id=1, cluster_id=AnalogOutput.cluster_id)
    .replaces(AnalogOutputCurtainAq2, endpoint_id=1)
    .replaces(BasicCluster, endpoint_id=1)
    .replaces(WindowCoveringCurtainAq2, endpoint_id=1)
    .add_to_registry()
)
