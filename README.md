# Laser Engine

Firmware and Python client for a multi-channel laser engine system built on Teensy 4.1. Controls 5 laser channels with temperature regulation via TCM (Thermo-Electric Cooler Module) controllers.

## Repository Structure

```
laser_engine/          Teensy 4.1 firmware + Python host client
  laser_engine.ino     Main firmware (state machine, TCM protocol, host protocol)
  pc-side-python.py    Python client for controlling the laser engine over USB serial
  compile.sh           Compile firmware
  upload.sh            Flash firmware to Teensy
tcm_args_download/     TCM parameter configuration tools
  tcm_args_download.ino  Firmware for TCM parameter setup
  pc-settings.py       Python client for TCM configuration
tests/                 Unit tests for the Python client
```

## Quick Start

### Firmware

Requires [arduino-cli](https://arduino.github.io/arduino-cli/) with the Teensy board package.

```bash
# Install dependencies
arduino-cli core install teensy:avr
arduino-cli lib install "CRC32" "elapsedMillis"

# Compile
arduino-cli compile -b teensy:avr:teensy41 ./laser_engine

# Upload (adjust port as needed)
arduino-cli upload -b teensy:avr:teensy41 ./laser_engine -p usb1/1-1
```

### Python Client

```bash
pip install -r requirements.txt

# Edit DEVICE or USBSN in pc-side-python.py to match your setup, then:
python laser_engine/pc-side-python.py
```

### Development

```bash
pip install -r requirements-dev.txt
pytest -v
```

## Architecture

The system has two main components:

**Firmware** (`laser_engine.ino`) — Runs on Teensy 4.1. Manages a state machine per temperature channel (WARMING_UP → ACTIVE → SLEEP, with error handling). Communicates with TCM modules over UART (Serial5) and with the host PC over USB serial. See [`laser_engine/README.md`](laser_engine/README.md) for detailed firmware documentation.

**Python Client** (`pc-side-python.py`) — Sends commands and receives status over USB serial. Supports querying status, waking/sleeping laser channels, and real-time monitoring with logging.

### Laser Channels

| Channel | Wavelength | Index |
|---------|-----------|-------|
| 405     | 402 nm    | 0     |
| 470     | 470 nm    | 1     |
| 638     | 638 nm    | 2     |
| 730     | 735 nm    | 3     |
| 55x     | 550 nm    | 4     |

### LED Status Indicators

| Color  | Meaning                          |
|--------|----------------------------------|
| Green  | All channels ACTIVE              |
| Blue   | At least one channel not ACTIVE  |
| Red    | At least one channel in ERROR    |
| Yellow | Interlock key is OFF             |

## License

MIT License
