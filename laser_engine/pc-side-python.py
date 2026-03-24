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
            raise ValueError("No device found with serial number or device")

        self.packet_serial = serial.Serial(ports[0], baudrate=baud_rate, timeout=1)

        self.lock = threading.RLock()
        self.query_interval = 1.0  # Query interval in seconds
        self.running = False
        self.query_thread = None
        self.thread_read_received_packet = None

        self.crc_mismatch = 0

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
            NUM_LASER_CH = 5
            NUM_TEMP_CH = 6
            BYTES_PER_CHANNEL = 7
            STATE_NAMES = ["WARMING_UP", "CHECK_ACTIVE", "ACTIVE", "WAKE_UP", "SLEEP", "PREPARE_SLEEP", "CHECK_ERROR", "ERROR"]

            laser_status = packet[1:1 + NUM_LASER_CH]
            temp_data = packet[1 + NUM_LASER_CH:-4]

            self.log_message('')
            self.log_message('New Round Query Data Start..')
            self.log_message("Laser TTL Status:" + str([bool(x) for x in laser_status]))

            for i in range(NUM_TEMP_CH):
                offset = i * BYTES_PER_CHANNEL
                state = temp_data[offset]
                temp = struct.unpack('>h', temp_data[offset + 1:offset + 3])[0] / 100.0
                tec_voltage = struct.unpack('>h', temp_data[offset + 3:offset + 5])[0] / 100.0
                tec_current = struct.unpack('>h', temp_data[offset + 5:offset + 7])[0] / 100.0

                state_str = STATE_NAMES[state] if state < len(STATE_NAMES) else f"UNKNOWN({state})"
                self.log_message(f"Channel {i}: State: {state_str}, Temp: {temp:.2f}°C, TEC Voltage: {tec_voltage:.2f}, TEC Current: {tec_current:.2f}")

            diff_temp_offset = NUM_TEMP_CH * BYTES_PER_CHANNEL
            for i in range(NUM_TEMP_CH):
                temp = struct.unpack('>h', temp_data[diff_temp_offset + i*2:diff_temp_offset + i*2 + 2])[0] / 100.0
                self.log_message(f"Channel {i}: DiffTemp: {temp:.2f}°C")

            hi_temp_offset = diff_temp_offset + NUM_TEMP_CH * 2
            for i in range(NUM_TEMP_CH):
                temp = struct.unpack('>h', temp_data[hi_temp_offset + i*2:hi_temp_offset + i*2 + 2])[0] / 100.0
                self.log_message(f"Channel {i}: Hi-Temp SetPoint: {temp:.2f}°C")

            self.log_message(f"CRC mismatch times: {self.crc_mismatch}")
        
        elif packet[0] == ord('A'):  # Acknowledgment packet
            print("Parameters set successfully")
        
        elif packet[0] == ord('N'):  # NAK packet
            print("Command not acknowledged")

        elif packet[0] == ord('G'):  # laser status packet 
            laser_channel = packet[1:2]
            laser_status = packet[2:3]
            self.log_message(f"Channel {laser_channel}: " + str([bool(x) for x in laser_status]))

    def query_loop(self):
        while self.running:
            self.query_status()
            time.sleep(self.query_interval)

    def received_loop(self):
        msg = []
        while self.running:
            if self.packet_serial.in_waiting == 0:
                continue

            char = self.packet_serial.read(1)
            if char == b'\r' and msg and msg[-1] == 0x0A:
                self.on_packet_received(bytearray(msg[:-1]))
                msg = []
                continue
            msg += char

    def start(self):
        self.running = True
        self.query_thread = threading.Thread(target=self.query_loop)
        self.query_thread.start()

        self.thread_read_received_packet = threading.Thread(target=self.received_loop)
        self.thread_read_received_packet.start()

    def stop(self):
        self.running = False
        self.packet_serial.close()
        if self.query_thread:
            self.query_thread.join()
        if self.thread_read_received_packet:
            self.thread_read_received_packet.join()

    def run(self):
        try:
            self.start()
            while True:
                self.query_status()
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.stop()

    def _send_packet(self, packet):
        '''Send a packet with CRC and terminator. Must be called with self.lock held.'''
        checksum = crc32(packet)
        self.packet_serial.write(packet + struct.pack('<I', checksum))
        self.packet_serial.write(b'\x0A\x0D')

    def query_status(self):
        '''
        API
        query all status information from firmware
        '''
        with self.lock:
            self._send_packet(b'Q')

    def wake_up(self, channel):
        '''
        API
        wake one channel from sleep status
        channel: 405, 470, 638, 735, 55x
        '''
        with self.lock:
            self._send_packet(b'W' + struct.pack('<I', self.mappings[channel]))

    def put_to_sleep(self, channel):
        '''
        API
        make one channel into sleep
        channel: 405, 470, 638, 735, 55x
        '''
        with self.lock:
            self._send_packet(b'S' + struct.pack('<I', self.mappings[channel]))

    def get_laser_status(self, channel):
        '''
        API
        get the channel status
        channel: 405, 470, 638, 735, 55x
        '''
        with self.lock:
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
