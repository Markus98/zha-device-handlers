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

        The device sends exactly two position reports per movement:
          1. An echo of the current position ~30 ms after the write is acknowledged.
          2. A final-position report when the curtain arrives at the target.

        These are distinguished by WindowCovering's _is_moving flag:

        - was_moving=True  → echo report: clear the flag and return.  The curtain is
          still moving; reads of present_value are unreliable mid-movement, so we do
          not update WindowCovering here.

        - was_moving=False → final (or periodic) report: accurate position data.
          Update WindowCovering twice with the same value in the same event-loop turn.
          ZHA's cover entity infers opening/closing direction from position jumps, so
          the first call may briefly set _is_opening/_is_closing internally; the
          second call at the same value immediately clears those flags before
          async_write_ha_state() runs, preventing a spurious "is opening/closing"
          state flash in Home Assistant.
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
            # Second call with the same value: ZHA sees "position unchanged" and
            # clears any opening/closing state it inferred from the first call.
            wc.update_attribute(wc_attr, wc_pos)


class WindowCoveringCurtainAq2(CustomCluster, WindowCovering):
    """Window covering cluster for handling motor commands."""

    _CONSTANT_ATTRIBUTES = {
        WindowCovering.AttributeDefs.window_covering_type.id: WindowCovering.WindowCoveringType.Rollershade,
    }

    def __init__(self, *args, **kwargs):
        """Init."""
        super().__init__(*args, **kwargs)
        # True between a movement command and the device's echo report.  Suppresses
        # the post-command position poll while the position read is known to be
        # unreliable (the motor is moving and reads return stale values).
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
          1. An echo of the current position (~30 ms after the write ACK).
          2. A final-position report when the curtain arrives.
        The echo clears _is_moving; the final report drives the WC update (x2).

        Reads of present_value are unreliable while the curtain is moving, so no
        explicit position read is done after movement or stop commands.  The device's
        own reports drive all position updates.

        _is_moving suppresses the post-command position poll while the echo has not
        yet arrived, preventing it from reading a stale position from the device.
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
            # Do not read position here, reads are unreliable while the motor is
            # decelerating.  The device sends a natural position report ~1 second
            # after stopping with the actual stopped position, which is accurate.
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

        While _is_moving is True the redirect is suppressed so that the post-command
        poll cannot read a stale position from the device before the echo report
        clears the flag.  The cached position (last known good value) is returned
        instead so the caller always gets a defined response for the attribute.
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
                # Return the last cached position rather than reading the device,
                # whose response would be unreliable mid-movement.  If nothing is
                # cached yet, surface a defined failure rather than silently
                # dropping the attribute from the response.
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
