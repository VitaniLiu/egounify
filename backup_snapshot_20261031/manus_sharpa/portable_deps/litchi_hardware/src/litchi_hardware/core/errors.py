"""Stable errors exposed by the HAL."""


class DeviceError(RuntimeError):
    """Base error for a device operation."""


class ConnectionError(DeviceError):
    """A device could not connect or lost its connection."""


class DependencyUnavailableError(DeviceError):
    """An optional vendor runtime dependency is not installed or loadable."""


class InvalidStateError(DeviceError):
    """An operation or lifecycle transition is invalid in the current state."""


class CommandRejectedError(DeviceError):
    """A well-formed command was rejected by validation or by the device."""


class ProtocolError(DeviceError):
    """External data did not conform to the expected protocol."""
