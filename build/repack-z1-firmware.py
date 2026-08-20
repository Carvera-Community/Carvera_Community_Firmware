#!/usr/bin/env python3
"""Replace the LPC1768 payload in a Makera Z1 firmware bundle."""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import struct
import tempfile
import zlib
from dataclasses import dataclass
from pathlib import Path


MAKERA_MAGIC = 0x4D5173EE
HEADER = struct.Struct("<IBBBBIIIIII")
VERSION_RE = re.compile(r"^(\d+)\.(\d+)\.(\d+)c[A-Za-z0-9._-]*$")
LPC_APP_START = 0x00004000
LPC_FLASH_END = 0x00080000


@dataclass(frozen=True)
class Bundle:
    header_version: int
    flags: int
    reserved: int
    esp_version: int
    lpc_version: int
    esp: bytes
    lpc: bytes


def parse_bundle(data: bytes) -> Bundle:
    if len(data) < HEADER.size:
        raise ValueError("bundle is shorter than its header")
    fields = HEADER.unpack_from(data)
    magic, header_version, header_length, flags, reserved = fields[:5]
    esp_size, lpc_size, esp_version, lpc_version, header_crc, file_crc = fields[5:]
    if magic != MAKERA_MAGIC:
        raise ValueError("bundle has invalid Makera magic")
    if header_version not in (1, 2):
        raise ValueError(f"unsupported bundle header version {header_version}")
    if header_length != HEADER.size:
        raise ValueError(f"unsupported bundle header length {header_length}")
    esp_present = esp_size != 0
    lpc_present = lpc_size != 0
    if bool(flags & 1) != esp_present or bool(flags & 2) != lpc_present:
        raise ValueError("bundle payload sizes disagree with its firmware flags")
    if not esp_present or not lpc_present:
        raise ValueError("bundle must contain both ESP and LPC firmware")
    if len(data) != header_length + esp_size + lpc_size:
        raise ValueError("bundle payload sizes do not match its file size")
    if zlib.crc32(data[:24]) & 0xFFFFFFFF != header_crc:
        raise ValueError("bundle header CRC does not match")
    if zlib.crc32(data[:28] + data[32:]) & 0xFFFFFFFF != file_crc:
        raise ValueError("bundle file CRC does not match")
    esp_start = header_length
    lpc_start = esp_start + esp_size
    return Bundle(
        header_version,
        flags,
        reserved,
        esp_version,
        lpc_version,
        data[esp_start:lpc_start],
        data[lpc_start:],
    )


def pack_version(version: str) -> int:
    match = VERSION_RE.fullmatch(version)
    if match is None:
        raise ValueError("VERSION must have the form major.minor.patchc[-suffix]")
    major, minor, patch = (int(part) for part in match.groups())
    if major > 0xFF or minor > 0xF or patch > 0xF:
        raise ValueError("VERSION components do not fit the Makera bundle format")
    return major << 24 | minor << 20 | patch << 16


def unpack_version(version: int) -> str:
    return f"{version >> 24}.{version >> 20 & 0xF}.{version >> 16 & 0xF}"


def validate_lpc_image(data: bytes, version: str) -> None:
    if len(data) < 8:
        raise ValueError("LPC image is shorter than its vector table")
    if len(data) > LPC_FLASH_END - LPC_APP_START:
        raise ValueError("LPC image does not fit the application flash region")
    initial_sp, reset_vector = struct.unpack_from("<II", data)
    stack_in_local_sram = 0x10000000 < initial_sp <= 0x10008000
    stack_in_ahb_sram = 0x2007C000 < initial_sp <= 0x20084000
    if initial_sp & 0x7 or not (stack_in_local_sram or stack_in_ahb_sram):
        raise ValueError("LPC image has an implausible initial stack pointer")
    reset_address = reset_vector & ~1
    if not reset_vector & 1 or not LPC_APP_START <= reset_address < LPC_APP_START + len(data):
        raise ValueError("LPC image has an implausible reset vector")
    if version.encode("ascii") + b"\0" not in data:
        raise ValueError("LPC image does not contain the requested VERSION")


def build_bundle(source: Bundle, lpc: bytes, lpc_version: int) -> bytes:
    values = (
        MAKERA_MAGIC,
        source.header_version,
        HEADER.size,
        source.flags,
        source.reserved,
        len(source.esp),
        len(lpc),
        source.esp_version,
        lpc_version,
    )
    header = HEADER.pack(*values, 0, 0)
    header_crc = zlib.crc32(header[:24]) & 0xFFFFFFFF
    header = HEADER.pack(*values, header_crc, 0)
    file_crc = zlib.crc32(header[:28] + source.esp + lpc) & 0xFFFFFFFF
    return HEADER.pack(*values, header_crc, file_crc) + source.esp + lpc


def write_atomic(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(data)
        os.chmod(temporary, 0o644)
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="original Makera firmware bundle")
    inspection = parser.add_mutually_exclusive_group()
    inspection.add_argument("--esp-version", action="store_true", help="print the bundled ESP version")
    inspection.add_argument("--bundle-id", action="store_true", help="print the bundle content hash")
    inspection.add_argument("--copy", type=Path, metavar="OUTPUT", help="validate and copy the original bundle")
    parser.add_argument("--lpc", type=Path, help="built LPC1768 main.bin")
    parser.add_argument("--version", help="community LPC firmware version")
    parser.add_argument("--output", type=Path, help="output bundle path")
    args = parser.parse_args()

    source_data = args.source.read_bytes()
    source = parse_bundle(source_data)
    if args.esp_version:
        print(unpack_version(source.esp_version))
        return
    if args.bundle_id:
        print(hashlib.sha256(source_data).hexdigest())
        return
    if args.copy is not None:
        write_atomic(args.copy, source_data)
        return
    if args.lpc is None or args.version is None or args.output is None:
        parser.error("--lpc, --version, and --output are required when repacking")
    lpc = args.lpc.read_bytes()
    lpc_version = pack_version(args.version)
    validate_lpc_image(lpc, args.version)
    result = build_bundle(source, lpc, lpc_version)
    parse_bundle(result)
    write_atomic(args.output, result)


if __name__ == "__main__":
    main()
