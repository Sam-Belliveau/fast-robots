"""Tests for the binary packet protocol (PacketWriter and PacketReader)."""

import struct

from main_python.ble.packet import DataType, PacketReader, PacketWriter


def test_ping_packet():
    w = PacketWriter(req_id=1, cmd_id=0)
    data = w.to_bytes()
    assert data == bytes([0x01, 0x00, 0x00])


def test_echo_packet_contains_string():
    w = PacketWriter(req_id=2, cmd_id=4)
    w.write("hello")
    data = w.to_bytes()
    # req_id(2) + cmd(1) + DT_STR(1) + len(1) + "hello"(5) = 10 bytes
    assert len(data) == 10
    assert data[3] == DataType.STR
    assert data[4] == 5
    assert data[5:10] == b"hello"


def test_motor_cmd_packet():
    w = PacketWriter(req_id=3, cmd_id=16)
    w.write(100)
    w.write(-50)
    data = w.to_bytes()
    # req_id(2) + cmd(1) + [DT_I32(1) + i32(4)] * 2 = 13 bytes
    assert len(data) == 13
    assert data[3] == DataType.I32
    assert struct.unpack_from("<i", data, 4)[0] == 100
    assert data[8] == DataType.I32
    assert struct.unpack_from("<i", data, 9)[0] == -50


def test_write_typed_explicit():
    w = PacketWriter(req_id=1, cmd_id=0)
    w.write_typed(DataType.I16, 42)
    data = w.to_bytes()
    assert data[3] == DataType.I16
    assert struct.unpack_from("<h", data, 4)[0] == 42


def test_read_pong_response():
    resp = struct.pack("<H", 1)
    resp += bytes([DataType.STR, 4]) + b"PONG"
    resp += bytes([DataType.END])

    r = PacketReader(resp)
    assert r.req_id == 1
    fields = r.read_all()
    assert fields == ["PONG"]
    assert r.has_end


def test_read_time_millis_response():
    resp = struct.pack("<H", 2)
    resp += bytes([DataType.I32]) + struct.pack("<i", 12345)
    resp += bytes([DataType.F32]) + struct.pack("<f", 72.5)
    resp += bytes([DataType.END])

    r = PacketReader(resp)
    assert r.req_id == 2
    fields = r.read_all()
    assert fields[0] == 12345
    assert abs(fields[1] - 72.5) < 0.01
    assert r.has_end


def test_read_without_end_marker():
    resp = struct.pack("<H", 5)
    resp += bytes([DataType.I32]) + struct.pack("<i", 999)

    r = PacketReader(resp)
    fields = r.read_all()
    assert fields == [999]
    assert not r.has_end


def test_read_empty_ack():
    resp = struct.pack("<H", 10)
    resp += bytes([DataType.END])

    r = PacketReader(resp)
    fields = r.read_all()
    assert fields == []
    assert r.has_end


def test_multi_packet_accumulation():
    """Simulate two notification packets for the same req_id."""
    # First packet: two i32 values, no DT_END
    pkt1 = struct.pack("<H", 7)
    pkt1 += bytes([DataType.I32]) + struct.pack("<i", 100)
    pkt1 += bytes([DataType.F32]) + struct.pack("<f", 3.14)

    # Second packet: one more value + DT_END
    pkt2 = struct.pack("<H", 7)
    pkt2 += bytes([DataType.I32]) + struct.pack("<i", 200)
    pkt2 += bytes([DataType.F32]) + struct.pack("<f", 6.28)
    pkt2 += bytes([DataType.END])

    r1 = PacketReader(pkt1)
    assert r1.req_id == 7
    accumulated = r1.read_all()
    assert not r1.has_end

    r2 = PacketReader(pkt2)
    assert r2.req_id == 7
    accumulated.extend(r2.read_all())
    assert r2.has_end

    assert len(accumulated) == 4
    assert accumulated[0] == 100
    assert abs(accumulated[1] - 3.14) < 0.01
    assert accumulated[2] == 200
    assert abs(accumulated[3] - 6.28) < 0.01
