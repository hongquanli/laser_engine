"""Tests for TeensyController packet handling, CRC validation, and framing logic."""

import logging
import struct
import time
from unittest.mock import MagicMock, patch
from zlib import crc32

import pytest

# Import the module under test
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'laser_engine'))


def make_controller_no_serial():
    """Create a TeensyController with a mocked serial port, bypassing device discovery."""
    with patch('serial.tools.list_ports.comports') as mock_comports, \
         patch('serial.Serial') as mock_serial_cls:
        mock_port = MagicMock()
        mock_port.device = '/dev/ttyACM0'
        mock_port.serial_number = '12345'
        mock_comports.return_value = [mock_port]

        mock_serial = MagicMock()
        mock_serial.timeout = 1
        mock_serial_cls.return_value = mock_serial

        # Must import after patching
        from importlib import import_module
        mod = import_module('pc-side-python')
        controller = mod.TeensyController(device='/dev/ttyACM0')
        return controller, mock_serial


# We need to import the module with hyphens in name
@pytest.fixture
def controller_and_serial():
    """Fixture that provides a TeensyController with mocked serial port."""
    ctrl, ser = make_controller_no_serial()
    yield ctrl, ser


class TestSendPacket:
    """Tests for _send_packet CRC and framing."""

    def test_send_packet_writes_payload_crc_and_terminator(self, controller_and_serial):
        ctrl, mock_serial = controller_and_serial
        payload = b'Q'
        ctrl._send_packet(payload)

        expected_crc = crc32(payload)
        expected_data = payload + struct.pack('<I', expected_crc)
        expected_terminator = b'\x0A\x0D'

        calls = mock_serial.write.call_args_list
        assert len(calls) == 2
        assert calls[0][0][0] == expected_data
        assert calls[1][0][0] == expected_terminator

    def test_send_packet_with_channel_data(self, controller_and_serial):
        ctrl, mock_serial = controller_and_serial
        channel = struct.pack('<I', 2)
        payload = b'W' + channel
        ctrl._send_packet(payload)

        expected_crc = crc32(payload)
        expected_data = payload + struct.pack('<I', expected_crc)

        calls = mock_serial.write.call_args_list
        assert calls[0][0][0] == expected_data

    def test_send_packet_is_thread_safe(self, controller_and_serial):
        """Verify _send_packet acquires the lock."""
        ctrl, mock_serial = controller_and_serial
        lock_acquired = []

        original_write = mock_serial.write
        def tracking_write(data):
            lock_acquired.append(ctrl.lock._is_owned())
            return original_write(data)

        mock_serial.write = tracking_write
        ctrl._send_packet(b'Q')
        assert all(lock_acquired), "write() should be called while lock is held"


class TestApiMethods:
    """Tests for the public API methods (query_status, wake_up, etc.)."""

    def test_query_status_sends_Q_command(self, controller_and_serial):
        ctrl, mock_serial = controller_and_serial
        ctrl.query_status()

        payload = b'Q'
        expected_crc = struct.pack('<I', crc32(payload))
        data_written = mock_serial.write.call_args_list[0][0][0]
        assert data_written[0:1] == b'Q'
        assert data_written[1:] == expected_crc

    def test_wake_up_sends_W_with_channel(self, controller_and_serial):
        ctrl, mock_serial = controller_and_serial
        ctrl.wake_up('470')

        data_written = mock_serial.write.call_args_list[0][0][0]
        assert data_written[0:1] == b'W'
        channel = struct.unpack('<I', data_written[1:5])[0]
        assert channel == 1  # '470' maps to 1

    def test_put_to_sleep_sends_S_with_channel(self, controller_and_serial):
        ctrl, mock_serial = controller_and_serial
        ctrl.put_to_sleep('638')

        data_written = mock_serial.write.call_args_list[0][0][0]
        assert data_written[0:1] == b'S'
        channel = struct.unpack('<I', data_written[1:5])[0]
        assert channel == 2  # '638' maps to 2

    def test_get_laser_status_sends_G_with_channel(self, controller_and_serial):
        ctrl, mock_serial = controller_and_serial
        ctrl.get_laser_status('55x')

        data_written = mock_serial.write.call_args_list[0][0][0]
        assert data_written[0:1] == b'G'
        channel = struct.unpack('<I', data_written[1:5])[0]
        assert channel == 4  # '55x' maps to 4

    def test_channel_mappings_are_complete(self, controller_and_serial):
        ctrl, _ = controller_and_serial
        assert ctrl.mappings == {'405': 0, '470': 1, '638': 2, '730': 3, '55x': 4}


class TestOnPacketReceived:
    """Tests for on_packet_received parsing and CRC validation."""

    def _make_packet(self, payload):
        """Build a packet with valid CRC."""
        checksum = crc32(payload)
        return bytearray(payload + struct.pack('<I', checksum))

    def test_rejects_packet_shorter_than_4_bytes(self, controller_and_serial):
        ctrl, _ = controller_and_serial
        ctrl.on_packet_received(bytearray(b'\x00\x01\x02'))
        # Should return without error (no crash)

    def test_rejects_invalid_crc(self, controller_and_serial):
        ctrl, _ = controller_and_serial
        packet = bytearray(b'A' + b'\x00\x00\x00\x00')  # bad CRC
        ctrl.on_packet_received(packet)
        assert ctrl.crc_mismatch == 1

    def test_increments_crc_mismatch_counter(self, controller_and_serial):
        ctrl, _ = controller_and_serial
        bad_packet = bytearray(b'A' + b'\xff\xff\xff\xff')
        ctrl.on_packet_received(bad_packet)
        ctrl.on_packet_received(bad_packet)
        assert ctrl.crc_mismatch == 2

    def test_ack_packet_accepted(self, controller_and_serial):
        ctrl, _ = controller_and_serial
        packet = self._make_packet(b'A')
        ctrl.on_packet_received(packet)
        assert ctrl.crc_mismatch == 0

    def test_nak_packet_accepted(self, controller_and_serial):
        ctrl, _ = controller_and_serial
        packet = self._make_packet(b'N')
        ctrl.on_packet_received(packet)
        assert ctrl.crc_mismatch == 0

    def test_status_packet_parsing(self, controller_and_serial):
        """Build a valid status packet and verify it parses without error."""
        ctrl, _ = controller_and_serial

        NUM_LASER_CH = 5
        NUM_TEMP_CH = 6
        BYTES_PER_CHANNEL = 7

        payload = b'S'
        # Laser status (5 bytes)
        payload += bytes([0, 1, 0, 1, 0])
        # Temp channel data: 6 channels × 7 bytes each
        for i in range(NUM_TEMP_CH):
            state = 2  # ACTIVE
            temp = struct.pack('>h', 2500)  # 25.00°C
            voltage = struct.pack('>h', 330)  # 3.30V
            current = struct.pack('>h', 100)  # 1.00A
            payload += bytes([state]) + temp + voltage + current
        # Diff temp: 6 × 2 bytes
        for i in range(NUM_TEMP_CH):
            payload += struct.pack('>h', 10)  # 0.10°C
        # Hi-temp setpoint: 6 × 2 bytes
        for i in range(NUM_TEMP_CH):
            payload += struct.pack('>h', 5000)  # 50.00°C

        packet = self._make_packet(payload)
        # Should not raise
        ctrl.on_packet_received(packet)
        assert ctrl.crc_mismatch == 0

    def test_state_names_out_of_range(self, controller_and_serial):
        """Verify out-of-range state values don't crash the parser."""
        ctrl, _ = controller_and_serial

        NUM_LASER_CH = 5
        NUM_TEMP_CH = 6

        payload = b'S'
        payload += bytes([0] * NUM_LASER_CH)
        # First channel has invalid state=255
        for i in range(NUM_TEMP_CH):
            state = 255 if i == 0 else 2
            payload += bytes([state]) + struct.pack('>h', 2500) + struct.pack('>h', 330) + struct.pack('>h', 100)
        for i in range(NUM_TEMP_CH):
            payload += struct.pack('>h', 10)
        for i in range(NUM_TEMP_CH):
            payload += struct.pack('>h', 5000)

        packet = self._make_packet(payload)
        # Should not raise IndexError
        ctrl.on_packet_received(packet)


class TestReceivedLoopFraming:
    """Tests for the received_loop byte framing logic."""

    def test_valid_frame_dispatched(self, controller_and_serial):
        """A valid \n\r terminated frame should be dispatched."""
        ctrl, mock_serial = controller_and_serial

        # Build a simple ACK packet with framing
        payload = b'A'
        checksum = struct.pack('<I', crc32(payload))
        frame_data = payload + checksum + b'\x0A\x0D'

        # Mock read to return one byte at a time, then empty
        byte_iter = iter(frame_data)
        read_count = [0]

        def mock_read(n):
            try:
                b = bytes([next(byte_iter)])
                read_count[0] += 1
                return b
            except StopIteration:
                ctrl._running.clear()  # stop after frame
                return b''

        mock_serial.read = mock_read
        ctrl.on_packet_received = MagicMock()

        ctrl._running.set()
        ctrl.received_loop()

        # on_packet_received should have been called with the frame (minus \n)
        assert ctrl.on_packet_received.called
        received = ctrl.on_packet_received.call_args[0][0]
        expected = bytearray(payload + checksum)
        assert received == expected

    def test_stray_cr_discards_partial_data(self, controller_and_serial, caplog):
        """A \r without preceding \n should discard accumulated data with a warning."""
        ctrl, mock_serial = controller_and_serial

        # Send some garbage bytes then \r (without \n before it)
        frame_data = b'\x01\x02\x03\x0D'

        byte_iter = iter(frame_data)
        def mock_read(n):
            try:
                return bytes([next(byte_iter)])
            except StopIteration:
                ctrl._running.clear()
                return b''

        mock_serial.read = mock_read
        ctrl.on_packet_received = MagicMock()

        ctrl._running.set()
        with caplog.at_level(logging.WARNING):
            ctrl.received_loop()

        # on_packet_received should NOT have been called
        assert not ctrl.on_packet_received.called
        assert any("Discarding 3 bytes" in msg for msg in caplog.messages)

    def test_empty_cr_ignored(self, controller_and_serial):
        """A \r as the very first byte should not crash."""
        ctrl, mock_serial = controller_and_serial

        frame_data = b'\x0D'

        byte_iter = iter(frame_data)
        def mock_read(n):
            try:
                return bytes([next(byte_iter)])
            except StopIteration:
                ctrl._running.clear()
                return b''

        mock_serial.read = mock_read
        ctrl.on_packet_received = MagicMock()

        ctrl._running.set()
        ctrl.received_loop()

        assert not ctrl.on_packet_received.called

    def test_serial_exception_stops_loop(self, controller_and_serial):
        """SerialException should set _running clear and break."""
        ctrl, mock_serial = controller_and_serial
        import serial as pyserial

        mock_serial.read = MagicMock(side_effect=pyserial.SerialException("disconnected"))

        ctrl._running.set()
        ctrl.received_loop()

        assert not ctrl._running.is_set()


class TestStartStop:
    """Tests for start/stop lifecycle."""

    def test_start_sets_running(self, controller_and_serial):
        ctrl, mock_serial = controller_and_serial
        mock_serial.read = MagicMock(return_value=b'')

        ctrl.start()
        assert ctrl._running.is_set()
        assert ctrl.query_thread is not None
        assert ctrl.thread_read_received_packet is not None

        ctrl.stop()
        assert not ctrl._running.is_set()

    def test_stop_joins_threads_before_closing_port(self, controller_and_serial):
        """Verify stop() joins threads before closing the serial port."""
        ctrl, mock_serial = controller_and_serial
        mock_serial.read = MagicMock(return_value=b'')

        call_order = []
        original_close = mock_serial.close

        def track_close():
            call_order.append('close')
            return original_close()

        mock_serial.close = track_close

        ctrl.start()
        time.sleep(0.1)

        # Patch join to track ordering
        original_join_q = ctrl.query_thread.join
        original_join_r = ctrl.thread_read_received_packet.join

        def track_join_q(*args, **kwargs):
            call_order.append('join_query')
            return original_join_q(*args, **kwargs)

        def track_join_r(*args, **kwargs):
            call_order.append('join_receive')
            return original_join_r(*args, **kwargs)

        ctrl.query_thread.join = track_join_q
        ctrl.thread_read_received_packet.join = track_join_r

        ctrl.stop()

        assert call_order.index('join_query') < call_order.index('close')
        assert call_order.index('join_receive') < call_order.index('close')


class TestInit:
    """Tests for __init__ device discovery."""

    def test_raises_on_no_device(self):
        with patch('serial.tools.list_ports.comports', return_value=[]):
            from importlib import import_module
            mod = import_module('pc-side-python')
            with pytest.raises(ValueError, match="No device found"):
                mod.TeensyController(device='/dev/nonexistent')

    def test_finds_device_by_serial_number(self):
        with patch('serial.tools.list_ports.comports') as mock_comports, \
             patch('serial.Serial') as mock_serial_cls:
            mock_port = MagicMock()
            mock_port.device = '/dev/ttyACM0'
            mock_port.serial_number = 'ABC123'
            mock_comports.return_value = [mock_port]
            mock_serial_cls.return_value = MagicMock()

            from importlib import import_module
            mod = import_module('pc-side-python')
            ctrl = mod.TeensyController(SN='ABC123')
            mock_serial_cls.assert_called_with('/dev/ttyACM0', baudrate=115200, timeout=1)

    def test_finds_device_by_device_path(self):
        with patch('serial.tools.list_ports.comports') as mock_comports, \
             patch('serial.Serial') as mock_serial_cls:
            mock_port = MagicMock()
            mock_port.device = '/dev/ttyUSB0'
            mock_port.serial_number = None
            mock_comports.return_value = [mock_port]
            mock_serial_cls.return_value = MagicMock()

            from importlib import import_module
            mod = import_module('pc-side-python')
            ctrl = mod.TeensyController(device='/dev/ttyUSB0')
            mock_serial_cls.assert_called_with('/dev/ttyUSB0', baudrate=115200, timeout=1)
