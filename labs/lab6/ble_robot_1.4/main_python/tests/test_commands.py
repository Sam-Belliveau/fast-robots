"""Tests for command serialization and response parsing."""

import struct

from main_python.ble.packet import DataType, PacketWriter
from main_python.commands import (
    Ping,
    Echo,
    MotorCmd,
    GetTimeMillis,
    SendIMUData,
    SendToFData,
    SendPIDData,
    MotorCal,
    PIDGains,
    ToFStats,
    SendTimeMillis,
    TimeSample,
    IMUSample,
    ToFSample,
    ToFStatsSample,
    PIDSample,
)


class TestCommandSerialization:
    def test_ping_no_params(self):
        cmd = Ping()
        w = PacketWriter(1, cmd.cmd_id())
        cmd.write_params(w)
        data = w.to_bytes()
        # Only header: req_id(2) + cmd(1)
        assert len(data) == 3

    def test_echo_writes_string(self):
        cmd = Echo(msg="test")
        w = PacketWriter(1, cmd.cmd_id())
        cmd.write_params(w)
        data = w.to_bytes()
        assert data[3] == DataType.STR
        assert data[4] == 4
        assert data[5:9] == b"test"

    def test_motor_cmd_writes_two_ints(self):
        cmd = MotorCmd(left=150, right=-100)
        w = PacketWriter(1, cmd.cmd_id())
        cmd.write_params(w)
        data = w.to_bytes()
        assert struct.unpack_from("<i", data, 4)[0] == 150
        assert struct.unpack_from("<i", data, 9)[0] == -100

    def test_pid_gains_writes_three_floats(self):
        cmd = PIDGains(kp=1.5, ki=0.1, kd=0.05)
        w = PacketWriter(1, cmd.cmd_id())
        cmd.write_params(w)
        data = w.to_bytes()
        assert abs(struct.unpack_from("<f", data, 4)[0] - 1.5) < 0.001
        assert abs(struct.unpack_from("<f", data, 9)[0] - 0.1) < 0.001
        assert abs(struct.unpack_from("<f", data, 14)[0] - 0.05) < 0.001


class TestResponseParsing:
    def test_ping_parses_pong(self):
        cmd = Ping()
        result = cmd.parse_response(["PONG"])
        assert result == "PONG"

    def test_echo_joins_strings(self):
        cmd = Echo(msg="hi")
        result = cmd.parse_response(["Robot Says -> ", "hi"])
        assert result == "Robot Says -> hi"

    def test_get_time_millis(self):
        cmd = GetTimeMillis()
        result = cmd.parse_response([12345, 72.5])
        assert isinstance(result, TimeSample)
        assert result.time == 12345
        assert result.temp == 72.5

    def test_motor_cal_returns_float(self):
        cmd = MotorCal(cal=1.2)
        result = cmd.parse_response([1.2])
        assert abs(result - 1.2) < 0.001

    def test_send_time_millis_chunks(self):
        cmd = SendTimeMillis()
        fields = [100, 70.0, 200, 71.0, 300, 72.0]
        result = cmd.parse_response(fields)
        assert len(result) == 3
        assert result[0] == TimeSample(100, 70.0)
        assert result[2] == TimeSample(300, 72.0)

    def test_send_imu_data_chunks(self):
        cmd = SendIMUData()
        # One sample: time + 9 floats
        fields = [1000, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 10.0, 20.0, 30.0]
        result = cmd.parse_response(fields)
        assert len(result) == 1
        assert isinstance(result[0], IMUSample)
        assert result[0].time == 1000
        assert result[0].ax == 0.1
        assert result[0].mz == 30.0

    def test_send_tof_data_chunks(self):
        cmd = SendToFData()
        fields = [500, 150, 145, 1, 600, 200, 190, 2]
        result = cmd.parse_response(fields)
        assert len(result) == 2
        assert result[0] == ToFSample(500, 150, 145, 1)
        assert result[1] == ToFSample(600, 200, 190, 2)

    def test_tof_stats_chunks(self):
        cmd = ToFStats()
        fields = [1, 100, 304.5, 12.3, 2, 100, 500.2, 15.1]
        result = cmd.parse_response(fields)
        assert len(result) == 2
        assert isinstance(result[0], ToFStatsSample)
        assert result[0].sensor_id == 1
        assert result[1].sensor_id == 2

    def test_send_pid_data_chunks(self):
        cmd = SendPIDData()
        fields = [
            1000, 300, -10, 100, 90, 85, 50, 10, 5,
            2000, 280, -20, 120, 110, 105, 60, 12, 6,
        ]
        result = cmd.parse_response(fields)
        assert len(result) == 2
        assert isinstance(result[0], PIDSample)
        assert result[0].time == 1000
        assert result[0].error == -10
        assert result[0].pwm == 100
        assert result[0].motor_left == 90
        assert result[1].measurement == 280

    def test_none_response_commands(self):
        cmd = MotorCmd(left=0, right=0)
        assert cmd.parse_response([]) is None
