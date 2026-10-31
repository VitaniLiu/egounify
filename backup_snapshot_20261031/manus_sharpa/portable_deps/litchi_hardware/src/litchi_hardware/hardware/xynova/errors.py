"""Error handling for Xynova SDK calls.

All C entry points return ``XynovaErrorCode`` (a signed int32). ``0`` is success;
anything else raises :class:`XynovaError`. When the shared library itself cannot
be loaded (SDK not installed on the machine), we raise
:class:`XynovaSDKNotLoadedError` on the first attempted call, letting import-time
code (like ROS 2 launch validation) succeed in dry-run mode.
"""

from __future__ import annotations


class XynovaError(RuntimeError):
    """Raised when a Xynova C API function returns a non-zero error code."""

    def __init__(self, code: int, context: str = "") -> None:
        self.code = int(code)
        self.context = context
        name = _CODE_NAMES.get(self.code, f"UNKNOWN({self.code})")
        msg = f"Xynova SDK call failed: {name} (code={self.code})"
        if context:
            msg = f"{msg} [{context}]"
        super().__init__(msg)


class XynovaSDKNotLoadedError(RuntimeError):
    """Raised when SDK calls are attempted but ``libxynova`` cannot be resolved.

    Callers who want to work in dry-run mode should catch this at construction
    time and swap in a stub.
    """

    def __init__(self, detail: str) -> None:
        super().__init__(
            "Xynova SDK shared library is not available. "
            "Set XYNOVA_LIB to the absolute path of libxynova.so, "
            "or add its directory to LD_LIBRARY_PATH. "
            f"Underlying error: {detail}"
        )


# Values retained by the bundled vendor SDK 2.0.0 R3 ``xynova/c/error_code.h``.
_CODE_NAMES: dict[int, str] = {
    0: "XYNOVA_ERROR_CODE_OK",
    -1: "XYNOVA_ERROR_CODE_UNKNOWN",
    -2: "XYNOVA_ERROR_CODE_INVALID_ARG",
    -3: "XYNOVA_ERROR_CODE_NOT_IMPLEMENTED",
    -4: "XYNOVA_ERROR_CODE_TIMEOUT",
    -5: "XYNOVA_ERROR_CODE_NOT_FOUND",
    -6: "XYNOVA_ERROR_CODE_ALREADY_EXISTS",
    -7: "XYNOVA_ERROR_CODE_NO_MEMORY",
    -8: "XYNOVA_ERROR_CODE_IO",
    -9: "XYNOVA_ERROR_CODE_BUSY",
    -10: "XYNOVA_ERROR_CODE_INTERRUPTED",
    -11: "XYNOVA_ERROR_CODE_OUT_OF_RANGE",
    -12: "XYNOVA_ERROR_CODE_PERMISSION_DENIED",
    -13: "XYNOVA_ERROR_CODE_UNSUPPORTED",
    -14: "XYNOVA_ERROR_CODE_ALREADY_INITIALIZED",
    -15: "XYNOVA_ERROR_CODE_DATA_OUT_OF_DATE",
    -16: "XYNOVA_ERROR_CODE_NO_DATA_UPDATED",
    -1000: "XYNOVA_ERROR_CODE_LOG_SINK_ALREADY_EXISTS",
    -1001: "XYNOVA_ERROR_CODE_LOG_WRITER_ALREADY_EXISTS",
    -1002: "XYNOVA_ERROR_CODE_LOG_FORMATTER_ALREADY_EXISTS",
    -1003: "XYNOVA_ERROR_CODE_LOG_LOGGER_ALREADY_EXISTS",
    -1010: "XYNOVA_ERROR_CODE_LOG_SINK_NOT_FOUND",
    -1011: "XYNOVA_ERROR_CODE_LOG_WRITER_NOT_FOUND",
    -1012: "XYNOVA_ERROR_CODE_LOG_FORMATTER_NOT_FOUND",
    -1013: "XYNOVA_ERROR_CODE_LOG_LOGGER_NOT_FOUND",
    -1014: "XYNOVA_ERROR_CODE_LOG_LOGGER_CREATION_FAILED",
    -2000: "XYNOVA_ERROR_CODE_COMM_INIT_FAILED",
    -2001: "XYNOVA_ERROR_CODE_COMM_SHUTDOWN_FAILED",
    -2002: "XYNOVA_ERROR_CODE_COMM_INVALID_CONFIG",
    -2003: "XYNOVA_ERROR_CODE_COMM_ALREADY_INITIALIZED",
    -2004: "XYNOVA_ERROR_CODE_COMM_NOT_INITIALIZED",
    -2005: "XYNOVA_ERROR_CODE_COMM_NOT_LOOPING",
    -2006: "XYNOVA_ERROR_CODE_COMM_NO_MAPPING",
    -3000: "XYNOVA_ERROR_CODE_CAN_DRIVER_NOT_FOUND",
    -3001: "XYNOVA_ERROR_CODE_CAN_DRIVER_ALREADY_REGISTERED",
    -3002: "XYNOVA_ERROR_CODE_CAN_INVALID_VTABLE",
    -3003: "XYNOVA_ERROR_CODE_CAN_OPEN_FAILED",
    -3004: "XYNOVA_ERROR_CODE_CAN_CLOSE_FAILED",
    -3005: "XYNOVA_ERROR_CODE_CAN_SEND_FAILED",
    -3006: "XYNOVA_ERROR_CODE_CAN_RECV_FAILED",
    -3007: "XYNOVA_ERROR_CODE_CAN_SET_FILTERS_FAILED",
    -3008: "XYNOVA_ERROR_CODE_CAN_NOT_OPEN",
    -3009: "XYNOVA_ERROR_CODE_CAN_ALREADY_OPEN",
    -3010: "XYNOVA_ERROR_CODE_CAN_INVALID_CONFIG",
    -3011: "XYNOVA_ERROR_CODE_CAN_NO_DATA",
    -3012: "XYNOVA_ERROR_CODE_CAN_INVALID_VTABLE_NAME",
    -3013: "XYNOVA_ERROR_CODE_CAN_SET_BUFFER_FAILED",
    -3014: "XYNOVA_ERROR_CODE_CAN_NO_TX_BUFFERS",
    -3015: "XYNOVA_ERROR_CODE_CAN_REQUEST_FAILED",
    -3016: "XYNOVA_ERROR_CODE_CAN_REQUEST_VERIFY_FAILED",
    -3017: "XYNOVA_ERROR_CODE_CAN_ERROR_FRAME",
    -3018: "XYNOVA_ERROR_CODE_CAN_BUS_OFF",
    -3019: "XYNOVA_ERROR_CODE_CAN_BUS_ERROR",
    -4000: "XYNOVA_ERROR_CODE_SLAVE_CREATE_FAILED",
    -4001: "XYNOVA_ERROR_CODE_SLAVE_ADD_SCHEDULE_COMMAND_FAILED",
    -5000: "XYNOVA_ERROR_CODE_DEVICE_CREATE_FAILED",
    -5001: "XYNOVA_ERROR_CODE_DEVICE_CREATE_SLAVE_FAILED",
    -5002: "XYNOVA_ERROR_CODE_DEVICE_INIT_SCHEDULE_COMMAND_FAILED",
    -6000: "XYNOVA_ERROR_CODE_HAND_SET_MODE_FAILED",
    -6001: "XYNOVA_ERROR_CODE_HAND_SET_SYSTEM_ENABLEMENT_FAILED",
    -6002: "XYNOVA_ERROR_CODE_HAND_SET_BACK_TO_ZERO_FAILED",
    -6003: "XYNOVA_ERROR_CODE_HAND_CLEAR_ERRORS_FAILED",
    -6004: "XYNOVA_ERROR_CODE_HAND_NO_DATA_UPDATED",
    -6005: "XYNOVA_ERROR_CODE_HAND_DATA_OUT_OF_DATE",
    -6006: "XYNOVA_ERROR_CODE_HAND_CREATE_SLAVE_FAILED",
}


def check(ec: int, ctx: str = "") -> None:
    """Raise :class:`XynovaError` if ``ec`` is non-zero."""
    if ec != 0:
        raise XynovaError(ec, ctx)
