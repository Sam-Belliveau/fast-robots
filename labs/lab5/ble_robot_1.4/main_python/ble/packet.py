"""Binary BLE packet protocol matching lib_BLEPacket.h."""

import struct
from enum import IntEnum


class DataType(IntEnum):
    STR = 0x00
    I8 = 0x10
    I16 = 0x11
    I32 = 0x12
    U8 = 0x20
    U16 = 0x21
    U32 = 0x22
    F32 = 0x30
    END = 0xFF


# Maps DataType tag to (struct format char, byte size).
_TYPE_FORMAT = {
    DataType.I8: ("b", 1),
    DataType.I16: ("<h", 2),
    DataType.I32: ("<i", 4),
    DataType.U8: ("B", 1),
    DataType.U16: ("<H", 2),
    DataType.U32: ("<I", 4),
    DataType.F32: ("<f", 4),
}

# Python type to default DataType for PacketWriter.write() auto-detection.
_PY_TYPE_TO_DT = {
    int: DataType.I32,
    float: DataType.F32,
    str: DataType.STR,
}


class PacketWriter:
    """Builds an outgoing BLE command packet."""

    def __init__(self, req_id: int, cmd_id: int):
        self._buf = bytearray()
        self._buf += struct.pack("<H", req_id)
        self._buf += struct.pack("B", cmd_id)

    def write(self, value) -> None:
        """Auto-detect type and write a tagged value."""
        dt = _PY_TYPE_TO_DT.get(type(value))
        if dt is None:
            raise TypeError(f"Unsupported type: {type(value)}")
        self.write_typed(dt, value)

    def write_typed(self, dt: DataType, value) -> None:
        """Write a value with an explicit type tag."""
        if dt == DataType.STR:
            encoded = value.encode("utf-8") if isinstance(value, str) else value
            self._buf += struct.pack("B", dt)
            self._buf += struct.pack("B", len(encoded))
            self._buf += encoded
        else:
            fmt, _ = _TYPE_FORMAT[dt]
            self._buf += struct.pack("B", dt)
            self._buf += struct.pack(fmt, value)

    def to_bytes(self) -> bytes:
        return bytes(self._buf)


class PacketReader:
    """Parses an incoming BLE response packet."""

    def __init__(self, data: bytes | bytearray):
        self._data = bytes(data)
        self._pos = 0
        self.req_id = struct.unpack_from("<H", self._data, 0)[0]
        self._pos = 2
        self._found_end = False

    def read(self) -> int | float | str | None:
        """Read the next tagged value. Returns None at DT_END or end of data."""
        if self._found_end or self._pos >= len(self._data):
            return None

        tag = self._data[self._pos]
        self._pos += 1

        if tag == DataType.END:
            self._found_end = True
            return None

        if tag == DataType.STR:
            slen = self._data[self._pos]
            self._pos += 1
            s = self._data[self._pos : self._pos + slen].decode("utf-8")
            self._pos += slen
            return s

        dt = DataType(tag)
        fmt, size = _TYPE_FORMAT[dt]
        value = struct.unpack_from(fmt, self._data, self._pos)[0]
        self._pos += size
        return value

    def read_all(self) -> list:
        """Read all remaining fields until DT_END or end of data."""
        fields = []
        while True:
            v = self.read()
            if v is None:
                break
            fields.append(v)
        return fields

    @property
    def has_end(self) -> bool:
        return self._found_end
