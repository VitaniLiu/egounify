"""Minimal logging helper for the Xynova SDK.

The vendor's C++ ``LogManager`` is a full-featured async logger with sinks,
writers, and formatters. Our ROS 2 node prefers Python's own logging + ROS's
built-in logger, so we only bind the two entry points needed to (a) initialize
the SDK's global logger with a sane level and (b) route SDK log output somewhere
the ROS node can pick it up.

Extended sink/writer/formatter creation remains intentionally outside this
small facade; the bundled vendor ``log_api.h`` is available to applications
that need to bind those advanced objects.
"""

from __future__ import annotations

import ctypes as C
from collections.abc import Sequence
from enum import IntEnum

from litchi_hardware.hardware.xynova import _ffi
from litchi_hardware.hardware.xynova.errors import check


class LogLevel(IntEnum):
    TRACE = 0
    DEBUG = 1
    INFO = 2
    WARN = 3
    ERROR = 4
    FATAL = 5


class LogManager:
    """Thin facade over ``XynovaLogGlobalLogger_*``."""

    _initialized = False

    @classmethod
    def init_global(cls, level: LogLevel = LogLevel.INFO) -> None:
        _ffi.lib.XynovaLogGlobalLogger_init(int(level), None)
        cls._initialized = True

    @classmethod
    def set_level(cls, level: LogLevel) -> LogLevel:
        prev = _ffi.lib.XynovaLogGlobalLogger_setLogLevel(int(level))
        return LogLevel(prev)

    @classmethod
    def get_level(cls) -> LogLevel:
        return LogLevel(_ffi.lib.XynovaLogGlobalLogger_getLogLevel())

    @classmethod
    def init_default_objects(cls) -> None:
        check(
            _ffi.lib.XynovaLogObjectLogger_initDefaultObjects(), "LogManager.init_default_objects"
        )

    @classmethod
    def objects_initialized(cls) -> bool:
        return bool(_ffi.lib.XynovaLogObjectLogger_isInitialized())

    @classmethod
    def has_logger(cls, name: str) -> bool:
        return bool(_ffi.lib.XynovaLogObjectLogger_hasLogger(name.encode("utf-8")))

    @classmethod
    def create_logger(
        cls,
        name: str,
        *,
        level: LogLevel = LogLevel.INFO,
        sink_name: str = "default",
        writer_names: Sequence[str] = ("default",),
        lb_type: str = "LoggerBackend<BufferPoolBytes, ProducerConsumerQueue>",
        pool_size: int = 1024,
        block_size: int = 256,
        message_size: int = 256,
        flush_interval_us: int = 500_000,
        queue_size_multiple: float = 1.5,
    ) -> None:
        """Register a named logger the comm RT loop can resolve by ``logger_name``.

        An empty or unregistered ``logger_name`` makes the native comm thread
        dereference a null logger and crash (SIGSEGV), so call this once after
        :meth:`init_default_objects` and point every ``logger_name`` here.
        Defaults reuse the ``"default"`` sink/writer that method creates.
        """
        # Keep encoded buffers alive for the duration of the C call.
        writers_encoded = [w.encode("utf-8") for w in writer_names]
        writer_arr = (C.c_char_p * (len(writers_encoded) + 1))(
            *writers_encoded,
            None,  # NULL-terminated list
        )
        conf = _ffi.LogLoggerConfigC(
            sink_name=sink_name.encode("utf-8"),
            writer_name_list=writer_arr,
            lb_type=lb_type.encode("utf-8"),
            level=int(level),
            lb_config=_ffi.LogLoggerBackendConfigC(
                pool_size=pool_size,
                block_size=block_size,
                message_size=message_size,
                max_continuous_repeated_log=0xFFFFFFFF,
                flush_interval_us=flush_interval_us,
                queue_size_multiple=queue_size_multiple,
            ),
        )
        check(
            _ffi.lib.XynovaLogObjectLogger_createLogger(name.encode("utf-8"), C.byref(conf)),
            f"LogManager.create_logger({name!r})",
        )

    @classmethod
    def is_initialized(cls) -> bool:
        return cls._initialized
