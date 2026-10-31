"""Inspect and normalize ELF RPATH/RUNPATH entries without host-specific tools."""

from __future__ import annotations

import os
import struct
from dataclasses import dataclass
from pathlib import Path

_ELF_MAGIC = b"\x7fELF"
_ELFCLASS32 = 1
_ELFCLASS64 = 2
_ELFDATA2LSB = 1
_PT_LOAD = 1
_PT_DYNAMIC = 2
_DT_NULL = 0
_DT_STRTAB = 5
_DT_STRSZ = 10
_DT_RPATH = 15
_DT_RUNPATH = 29
_RUNTIME_PATH_TAGS = {_DT_RPATH, _DT_RUNPATH}


@dataclass(frozen=True, slots=True)
class _ProgramHeader:
    kind: int
    offset: int
    virtual_address: int
    file_size: int


@dataclass(frozen=True, slots=True)
class _DynamicTable:
    offset: int
    size: int
    entry_format: str
    entries: tuple[tuple[int, int], ...]
    program_headers: tuple[_ProgramHeader, ...]


def is_elf(path: Path) -> bool:
    """Return whether *path* starts with an ELF file signature."""

    if not path.is_file():
        return False
    with path.open("rb") as stream:
        return stream.read(len(_ELF_MAGIC)) == _ELF_MAGIC


def _unpack_from(
    data: bytes | bytearray, fmt: str, offset: int
) -> tuple[int, ...]:
    size = struct.calcsize(fmt)
    if offset < 0 or offset + size > len(data):
        raise ValueError("ELF structure extends beyond the end of the file")
    return struct.unpack_from(fmt, data, offset)


def _parse_dynamic_table(data: bytes | bytearray) -> _DynamicTable:
    if data[:4] != _ELF_MAGIC:
        raise ValueError("not an ELF file")
    if data[5] != _ELFDATA2LSB:
        raise ValueError("only little-endian ELF files are supported")
    if data[4] == _ELFCLASS64:
        program_header_offset = _unpack_from(data, "<Q", 32)[0]
        program_header_size = _unpack_from(data, "<H", 54)[0]
        program_header_count = _unpack_from(data, "<H", 56)[0]
        program_header_format = "<IIQQQQQQ"
        dynamic_entry_format = "<qQ"
    elif data[4] == _ELFCLASS32:
        program_header_offset = _unpack_from(data, "<I", 28)[0]
        program_header_size = _unpack_from(data, "<H", 42)[0]
        program_header_count = _unpack_from(data, "<H", 44)[0]
        program_header_format = "<IIIIIIII"
        dynamic_entry_format = "<iI"
    else:
        raise ValueError("unsupported ELF class")

    expected_program_header_size = struct.calcsize(program_header_format)
    if program_header_size < expected_program_header_size:
        raise ValueError("ELF program header is smaller than expected")

    program_headers = []
    dynamic_header: _ProgramHeader | None = None
    for index in range(program_header_count):
        offset = program_header_offset + index * program_header_size
        values = _unpack_from(data, program_header_format, offset)
        header = _ProgramHeader(
            kind=values[0],
            offset=values[2] if data[4] == _ELFCLASS64 else values[1],
            virtual_address=values[3] if data[4] == _ELFCLASS64 else values[2],
            file_size=values[5] if data[4] == _ELFCLASS64 else values[4],
        )
        program_headers.append(header)
        if header.kind == _PT_DYNAMIC:
            dynamic_header = header
    if dynamic_header is None:
        return _DynamicTable(
            0, 0, dynamic_entry_format, (), tuple(program_headers)
        )

    entry_size = struct.calcsize(dynamic_entry_format)
    entries = []
    for offset in range(
        dynamic_header.offset,
        dynamic_header.offset + dynamic_header.file_size,
        entry_size,
    ):
        tag, value = _unpack_from(data, dynamic_entry_format, offset)
        entries.append((tag, value))
        if tag == _DT_NULL:
            break
    if not entries or entries[-1][0] != _DT_NULL:
        raise ValueError("ELF dynamic table has no terminating DT_NULL entry")
    return _DynamicTable(
        dynamic_header.offset,
        dynamic_header.file_size,
        dynamic_entry_format,
        tuple(entries),
        tuple(program_headers),
    )


def _virtual_to_file_offset(
    address: int, program_headers: tuple[_ProgramHeader, ...]
) -> int:
    for header in program_headers:
        if (
            header.kind == _PT_LOAD
            and header.virtual_address
            <= address
            < header.virtual_address + header.file_size
        ):
            return header.offset + address - header.virtual_address
    raise ValueError(f"ELF virtual address 0x{address:x} is not file-backed")


def runtime_paths(path: Path) -> tuple[str, ...]:
    """Return the decoded DT_RPATH and DT_RUNPATH values from *path*."""

    data = path.read_bytes()
    table = _parse_dynamic_table(data)
    if not table.entries:
        return ()
    values = dict(table.entries)
    string_table_address = values.get(_DT_STRTAB)
    string_table_size = values.get(_DT_STRSZ)
    if string_table_address is None or string_table_size is None:
        if any(tag in _RUNTIME_PATH_TAGS for tag, _ in table.entries):
            raise ValueError("ELF runtime path has no associated string table")
        return ()
    string_table_offset = _virtual_to_file_offset(
        string_table_address, table.program_headers
    )

    result = []
    for tag, value in table.entries:
        if tag not in _RUNTIME_PATH_TAGS:
            continue
        if value >= string_table_size:
            raise ValueError("ELF runtime path points outside its string table")
        start = string_table_offset + value
        end = data.find(b"\0", start, string_table_offset + string_table_size)
        if end < 0:
            raise ValueError("ELF runtime path is not null-terminated")
        result.append(data[start:end].decode("utf-8"))
    return tuple(result)


def remove_runtime_paths(path: Path) -> None:
    """Remove DT_RPATH and DT_RUNPATH entries from an ELF file in place."""

    data = bytearray(path.read_bytes())
    table = _parse_dynamic_table(data)
    if not any(tag in _RUNTIME_PATH_TAGS for tag, _ in table.entries):
        return

    kept = [
        (tag, value)
        for tag, value in table.entries
        if tag not in _RUNTIME_PATH_TAGS and tag != _DT_NULL
    ]
    entries = [*kept, (_DT_NULL, 0)]
    entry_size = struct.calcsize(table.entry_format)
    slot_count = table.size // entry_size
    if len(entries) > slot_count:
        raise ValueError("normalized ELF dynamic table does not fit")
    entries.extend([(_DT_NULL, 0)] * (slot_count - len(entries)))
    for index, entry in enumerate(entries):
        struct.pack_into(
            table.entry_format, data, table.offset + index * entry_size, *entry
        )

    mode = path.stat().st_mode
    temporary = path.with_name(f".{path.name}.rpath-{os.getpid()}")
    try:
        temporary.write_bytes(data)
        temporary.chmod(mode)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def assert_relocatable_runtime_paths(path: Path) -> None:
    """Reject runtime search paths that depend on a build host or working directory."""

    invalid = []
    for value in runtime_paths(path):
        for entry in value.split(":"):
            if not (
                entry == "$ORIGIN"
                or entry.startswith("$ORIGIN/")
                or entry == "${ORIGIN}"
                or entry.startswith("${ORIGIN}/")
            ):
                invalid.append(entry or "<empty>")
    if invalid:
        joined = ", ".join(invalid)
        raise ValueError(f"{path} has non-relocatable ELF runtime paths: {joined}")


def validate_elf_tree(root: Path) -> None:
    """Validate every ELF file below *root*."""

    for path in sorted(root.rglob("*")):
        if is_elf(path):
            assert_relocatable_runtime_paths(path)
