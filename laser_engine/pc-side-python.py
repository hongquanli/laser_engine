#! /usr/bin/env python3
# coding=utf-8

"""
 description:
 author:		kevin.wang
 create date:	2024-07-17
 version:		1.0.0
"""

import struct
import time
import threading
import logging
from zlib import crc32

import serial
from serial.tools import list_ports

logging.basicConfig(filename='laser_engine.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

#USBSN = '12769670'
USBSN = None
DEVICE = "/dev/ttyACM0"

#USBSN = '12769670'
#DEVICE = None

class TeensyController:
    def __init__(self, SN=None, device=None, baud_rate=115200):
        ports = []
        for p in list_ports.comports():
            if SN is not None:
                if SN == p.serial_number:
                    ports.append(p.device)
            elif device is not None:
                if device == p.device:
                    ports.append(p.device)
        
        if not ports:
            wanted = f"serial number {SN!r}" if SN is not None else f"device {device!r}"
            attached = ", ".join(
                f"{p.device} (SN {p.serial_number})" for p in list_ports.comports()
            ) or "none"
            raise ValueError(f"No device found with {wanted}. Attached ports: {attached}")

        self.port = ports[0]
        self.packet_serial = serial.Serial(self.port, baudrate=baud_rate, timeout=1)

        self.lock = threading.RLock()
        self.query_interval = 1.0  # Query interval in seconds
        self._running = threading.Event()
        self.query_thread = None
        self.thread_read_received_packet = None

        self.crc_mismatch = 0

        # Optional hook: called with the dict from parse_status_packet() every time
        # a status packet arrives. Used by the GUI; None means terminal-only operation.
        # Runs on the serial reader thread.
        self.status_callback = None

        # Wake/sleep targets. Firmware reports 6 thermal channels but exposes 5
        # lines here: channel 4 is the 555nm NECSEL diode and channel 5 is its
        # PPLN doubling-crystal oven, so '55x' addresses both.
        self.mappings = {
            '405': 0,
            '470': 1,
            '638': 2,
            '730': 3,
            '55x': 4,
        }

    def log_message(self, message):
        """
        Logs a message to the console and a file.

        :param message: The message to be logged.
        """
        logging.info(message)
        print(message)  # Optional: Print to console for real-time feedback

    NUM_LASER_CH = 5
    NUM_TEMP_CH = 6
    BYTES_PER_CHANNEL = 7
    STATE_NAMES = ["WARMING_UP", "CHECK_ACTIVE", "ACTIVE", "WAKE_UP", "SLEEP", "PREPARE_SLEEP", "CHECK_ERROR", "ERROR"]

    def parse_status_packet(self, packet):
        '''
        Decode an 'S' status packet into a dict:

            {'laser_ttl': [bool x5],
             'channels': [{'state', 'state_str', 'temp', 'tec_voltage',
                           'tec_current', 'diff_temp', 'hi_temp_setpoint'} x6]}

        CRC is assumed already verified by the caller.
        '''
        laser_ttl = [bool(x) for x in packet[1:1 + self.NUM_LASER_CH]]
        temp_data = packet[1 + self.NUM_LASER_CH:-4]
        diff_temp_offset = self.NUM_TEMP_CH * self.BYTES_PER_CHANNEL
        hi_temp_offset = diff_temp_offset + self.NUM_TEMP_CH * 2

        def centi(offset):
            return struct.unpack('>h', temp_data[offset:offset + 2])[0] / 100.0

        channels = []
        for i in range(self.NUM_TEMP_CH):
            offset = i * self.BYTES_PER_CHANNEL
            state = temp_data[offset]
            channels.append({
                'state': state,
                'state_str': self.STATE_NAMES[state] if state < len(self.STATE_NAMES) else f"UNKNOWN({state})",
                'temp': centi(offset + 1),
                'tec_voltage': centi(offset + 3),
                'tec_current': centi(offset + 5),
                'diff_temp': centi(diff_temp_offset + i*2),
                'hi_temp_setpoint': centi(hi_temp_offset + i*2),
            })

        return {'laser_ttl': laser_ttl, 'channels': channels}

    def on_packet_received(self, packet):
        if len(packet) < 4:
            return

        received_crc = struct.unpack('<I', packet[-4:])[0]
        calculated_crc = crc32(packet[:-4])

        if received_crc != calculated_crc:
            self.crc_mismatch += 1
            print("CRC mismatch")
            return

        if packet[0] == ord('S'):  # Status packet
            status = self.parse_status_packet(packet)

            self.log_message('')
            self.log_message('New Round Query Data Start..')
            self.log_message("Laser TTL Status:" + str(status['laser_ttl']))

            for i, ch in enumerate(status['channels']):
                self.log_message(f"Channel {i}: State: {ch['state_str']}, Temp: {ch['temp']:.2f}°C, TEC Voltage: {ch['tec_voltage']:.2f}, TEC Current: {ch['tec_current']:.2f}")

            for i, ch in enumerate(status['channels']):
                self.log_message(f"Channel {i}: DiffTemp: {ch['diff_temp']:.2f}°C")

            for i, ch in enumerate(status['channels']):
                self.log_message(f"Channel {i}: Hi-Temp SetPoint: {ch['hi_temp_setpoint']:.2f}°C")

            self.log_message(f"CRC mismatch times: {self.crc_mismatch}")

            if self.status_callback is not None:
                self.status_callback(status)

        elif packet[0] == ord('A'):  # Acknowledgment packet
            print("Parameters set successfully")
        
        elif packet[0] == ord('N'):  # NAK packet
            print("Command not acknowledged")

        elif packet[0] == ord('G'):  # laser status packet 
            laser_channel = packet[1:2]
            laser_status = packet[2:3]
            self.log_message(f"Channel {laser_channel}: " + str([bool(x) for x in laser_status]))

    def query_loop(self):
        while self._running.is_set():
            try:
                self.query_status()
            except serial.SerialException as e:
                logging.error(f"Serial error in query loop: {e}")
                self._running.clear()
                break
            except (OSError, ValueError, struct.error) as e:
                logging.error(f"Unexpected error in query loop: {e}")
            time.sleep(self.query_interval)

    def received_loop(self):
        msg = []
        while self._running.is_set():
            try:
                char = self.packet_serial.read(1)
                if not char:
                    continue
                if char == b'\r':
                    if msg and msg[-1] == 0x0A:
                        self.on_packet_received(bytearray(msg[:-1]))
                    elif msg:
                        logging.warning(f"Discarding {len(msg)} bytes of partial packet data")
                    msg = []
                    continue
                msg += char
            except serial.SerialException as e:
                logging.error(f"Serial error in receive loop: {e}")
                self._running.clear()
                break
            except (OSError, ValueError, struct.error) as e:
                logging.error(f"Error processing received data: {e}")
                msg = []

    def start(self):
        self._running.set()
        self.query_thread = threading.Thread(target=self.query_loop)
        self.query_thread.start()

        self.thread_read_received_packet = threading.Thread(target=self.received_loop)
        self.thread_read_received_packet.start()

    def stop(self):
        self._running.clear()
        if self.query_thread:
            self.query_thread.join()
        if self.thread_read_received_packet:
            self.thread_read_received_packet.join()
        self.packet_serial.close()

    def run(self):
        try:
            self.start()
            while self._running.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.stop()

    def _send_packet(self, packet):
        '''Send a packet with CRC and terminator. Thread-safe.'''
        with self.lock:
            checksum = crc32(packet)
            self.packet_serial.write(packet + struct.pack('<I', checksum))
            self.packet_serial.write(b'\x0A\x0D')

    def query_status(self):
        '''
        API
        query all status information from firmware
        '''
        self._send_packet(b'Q')

    def wake_up(self, channel):
        '''
        API
        wake one channel from sleep status
        channel: 405, 470, 638, 735, 55x
        '''
        self._send_packet(b'W' + struct.pack('<I', self.mappings[channel]))

    def put_to_sleep(self, channel):
        '''
        API
        make one channel into sleep
        channel: 405, 470, 638, 735, 55x
        '''
        self._send_packet(b'S' + struct.pack('<I', self.mappings[channel]))

    def get_laser_status(self, channel):
        '''
        API
        get the channel status
        channel: 405, 470, 638, 735, 55x
        '''
        self._send_packet(b'G' + struct.pack('<I', self.mappings[channel]))


if __name__ == "__main__":
    # Adjust port or device as needed
    controller = TeensyController(device=DEVICE, SN=USBSN)
    
    # Example usage in a separate thread
    def set_parameters_thread():
        time.sleep(2)

        # channel: 405, 470, 638, 735, 55x
        # controller.put_to_sleep('55x')
        # controller.wake_up('55x')

        # the information display in the on_packet_received function
        #controller.get_status('55x')

    parameter_thread = threading.Thread(target=set_parameters_thread)
    parameter_thread.start()

    controller.run()
