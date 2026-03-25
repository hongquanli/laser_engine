/*!
  \file    laser.ino 
  \brief   modif for test laser

  \version 2024-07-17, V1.0
	\author	 kevin.wang
	\note    none
*/
#include <cstdlib>
#include <cstdio>

#include <FastLED.h>

#include <elapsedMillis.h>
#include <CRC32.h>

// Teensy4.1 board v2 def

// RGB LED define
static const int LED_R = 36;
static const int LED_G = 37;
static const int LED_B = 38;

// laser driver en
static const int LASER_550nm = 19;
static const int LASER_402nm = 18;
static const int LASER_470nm = 17;
static const int LASER_638nm = 16;
static const int LASER_735nm = 15;

// despeckler
static const int Dispecker_pin   = 33;

// laser status read port
static const int STATUS_402nm_TTL = 31;
static const int STATUS_470nm_TTL = 30;
static const int STATUS_550nm_TTL = 32;
static const int STATUS_638nm_TTL = 29;
static const int STATUS_735nm_TTL = 28;

// true: would display the key status
static const bool config_enable_key_status_led = false;
// the pin status of KEY
static const int Interlock_pin = 9;

const int NUM_LASER_CHANNELS = 5;
const int NUM_TEMP_CHANNELS = 6;
const int NUM_VOLTAGE_CHANNELS = 6;
const int NUM_CURRENT_CHANNELS = 6;
const unsigned long CHANNELS_SLEEP_TIMEOUT[NUM_TEMP_CHANNELS] = {
  3 * 60UL * 60UL * 1000UL, 
  3 * 60UL * 60UL * 1000UL, 
  3 * 60UL * 60UL * 1000UL, 
  3 * 60UL * 60UL * 1000UL, 
  30UL * 60UL * 1000UL, 
  30UL * 60UL * 1000UL
};

unsigned long lastActiveTime[NUM_TEMP_CHANNELS] = {0};
bool lastActiveTimeRecordFlag[NUM_TEMP_CHANNELS] = {false};

// If due to worktime is over 8 hours, the wakeupCoolDownTime would be 10 minutes
// others is 1 seconds
const unsigned long SHORT_WAKEUP_COOLDOWN_TIME = 1UL * 1000UL;
unsigned long wakeupCoolDownTime[NUM_TEMP_CHANNELS] = {
  SHORT_WAKEUP_COOLDOWN_TIME,
  SHORT_WAKEUP_COOLDOWN_TIME,
  SHORT_WAKEUP_COOLDOWN_TIME,
  SHORT_WAKEUP_COOLDOWN_TIME,
  SHORT_WAKEUP_COOLDOWN_TIME,
  SHORT_WAKEUP_COOLDOWN_TIME
};
unsigned long recordSleepTime[NUM_TEMP_CHANNELS];

const float TEMP_ERROR_THRESHOLD = 5;
const unsigned long ERROR_DURATION = 5000UL; // 5 seconds in milliseconds
const unsigned long ACTIVE_DURATION = 5000UL; // 5 seconds in milliseconds

const unsigned long EACH_QUERY_DISTANCE = 200UL; // 200 in milliseconds

enum CommandType{
	NONE = 0,
  TEMPERATURE = 1,
 	VOLTAGE = 2,
	CURRENT = 3,
  HITEMPSETPOINT = 4,
  ADJUSTTEMP = 5,
  ENABLETCM = 6,
  SWITCHTCMSTATUS
};

const uint8_t MAX_QUERY_TYPE_INDEX = 11;
CommandType query_frequnce[MAX_QUERY_TYPE_INDEX] = { TEMPERATURE, VOLTAGE, CURRENT, TEMPERATURE, VOLTAGE, TEMPERATURE, VOLTAGE, TEMPERATURE, CURRENT, HITEMPSETPOINT, SWITCHTCMSTATUS };
uint8_t current_query_type_index = 0;

const int8_t ERR_OUT_OF_RANGE = 100;

/*
 *
 * PREPARE_SLEEP: (middle status) do the action before enter real SLEEP status
 * WAKE_UP: (middle status) do the action for return status from SLEEP to WARMING_UP
 * CHECK_ERROR: (middle status) before enter ERROR status, do some check for confirming
 *
 */
enum ChannelState {
  WARMING_UP,
  CHECK_ACTIVE,
  ACTIVE,
  WAKE_UP,
  SLEEP,
  PREPARE_SLEEP,
  CHECK_ERROR,
  ERROR
};

CRC32 crc;
// Pins for laser channels (adjust as needed)
const int laserPins[NUM_LASER_CHANNELS] = {LASER_402nm, LASER_470nm, LASER_638nm, LASER_735nm, LASER_550nm};

// Add a global array to track laser pin states, avoid to disable or enable lasers status too many times
// false: means the laser pin is at disable action status 
// true: means the laser pin is at enable action status 
bool laserPinActionStates[NUM_LASER_CHANNELS] = {false};

// read the real laser status pins
const int laserStatuspins[NUM_LASER_CHANNELS] = {STATUS_402nm_TTL, STATUS_470nm_TTL, STATUS_638nm_TTL, STATUS_735nm_TTL, STATUS_550nm_TTL};

uint8_t laserStatus[NUM_LASER_CHANNELS] = {0};
unsigned long lastLaserStatusChangeTime[NUM_LASER_CHANNELS] = {0};
elapsedMillis timeSinceLastQueryLaserStatus = 0;

// Temperature setpoints and channel states
float tempSetpoints[NUM_TEMP_CHANNELS] = {25.0, 25.0, 25.0, 25.0, 25.0, 99.7};
// indicate the channels' adjust temperature whether have been queried
bool getTempReady[NUM_TEMP_CHANNELS] = {false};
bool enableTCMReady[NUM_TEMP_CHANNELS] = {false};

float tempCurrentPoints[NUM_TEMP_CHANNELS] = {0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
float highTempCurrentPoints[NUM_TEMP_CHANNELS] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

// Voltage value of channels
float voltageCurrentPoints[NUM_VOLTAGE_CHANNELS] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

// current value of channels
float currentCurrentPoints[NUM_CURRENT_CHANNELS] = {0, 0, 0, 0, 0.1, 0.2};

ChannelState channelStates[NUM_TEMP_CHANNELS] = {WARMING_UP};
unsigned long lastChannelStatesChangeTime[NUM_TEMP_CHANNELS] = {0};

elapsedMillis timeInErrorState[NUM_TEMP_CHANNELS] = {0};

// when status changes, need to wait some time to confirm device status
elapsedMillis timeStatusCheck[NUM_TEMP_CHANNELS] = {0};

elapsedMillis timeSinceLastQueryTemp = 0;
uint8_t gQueryTemperatureChannelIndex = 0;
uint8_t gQueryVoltageChannelIndex = 0;
uint8_t gQueryCurrentChannelIndex = 0;
uint8_t gQueryHiTempSetPointChannelIndex = 0;
uint8_t gQueryAdjustTempChannelIndex = 0;
uint8_t gEnableTCMChannelIndex = 0;
uint8_t gProcessChannelStatusIndex = 0;

// TCM modules protocol process variables
bool reply_frame_analyzing_flag = false;

char tcm_command_buf[256];
uint16_t tcm_command_buf_length = 0;

char tcm_reply_buf[256];
uint16_t tcm_reply_buf_length = 0;

// for judge whether the replay value is correct or not
char tcm_reply_title[256];
uint8_t tcm_reply_title_length = 0;
uint8_t tcm_reply_address = 0;
uint8_t tcm_reply_module = 0;
CommandType tcm_reply_command_type = NONE;

// host protocol process variables
uint8_t host_protocol_buf[256];
uint16_t host_protocol_buf_length = 0;

uint8_t key_status = 0;
uint32_t tcm_parse_failure_count = 0;

enum LEDState {
  RED,
  BLUE,
  GREEN,
  YELLOW
};

LEDState led_state = GREEN;
void set_status_LED(LEDState status) {
	switch (status) {
		case RED:
			if (led_state == RED)
				return;
  		digitalWrite(LED_R, HIGH);
  		digitalWrite(LED_G, LOW);
  		digitalWrite(LED_B, LOW);
			led_state = RED;
			break;

		case BLUE:
			if (led_state == BLUE)
				return;
  		digitalWrite(LED_R, LOW);
  		digitalWrite(LED_G, LOW);
  		digitalWrite(LED_B, HIGH);
			led_state = BLUE;
			break;

		case GREEN:
			if (led_state == GREEN)
				return;
  		digitalWrite(LED_R, LOW);
  		digitalWrite(LED_G, HIGH);
  		digitalWrite(LED_B, LOW);
			led_state = GREEN;
			break;

    case YELLOW:
      if (led_state == YELLOW)
        return;
  		digitalWrite(LED_R, HIGH);
  		digitalWrite(LED_G, HIGH);
  		digitalWrite(LED_B, LOW);
			led_state = YELLOW;
			break;

		default:
			break;
	}
}

void indicate_device_status() {
  if (config_enable_key_status_led) {
    if (key_status == 0) {
      set_status_LED(YELLOW);
      return;
    }
  }

  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    if (channelStates[i] == ERROR) {
      set_status_LED(RED);
      return;
    }
  }

  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    if (channelStates[i] != ACTIVE) {
      set_status_LED(BLUE);
      return;
    }
  }

  // all channels are ACTIVE
  set_status_LED(GREEN);
}

void setParameters(const uint8_t* buffer, size_t size) {
  if (size == 1 + NUM_TEMP_CHANNELS * sizeof(float)) {
    for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
      memcpy(&tempSetpoints[i], buffer + 1 + i * sizeof(float), sizeof(float));
    }
    sendACK();
  } else {
    sendNAK();
  }
}

void disableLaser(int channel) {
	// 5 channels laser and 6 temperature channels
	if (channel >= NUM_LASER_CHANNELS)
		channel = NUM_LASER_CHANNELS - 1;

	if (channel < NUM_LASER_CHANNELS) {
		// Only write if the pin is not already disabled
		if (laserPinActionStates[channel] != false) {
			digitalWrite(laserPins[channel], HIGH);
			laserPinActionStates[channel] = false; // Update state
		}
	}
}

void enableLaser(int channel) {
	// 5 channels laser and 6 temperature channels
	if (channel >= NUM_LASER_CHANNELS)
		channel = NUM_LASER_CHANNELS - 1;

	if (channel < NUM_LASER_CHANNELS) {
		// Only write if the pin is not already enabled
		if (laserPinActionStates[channel] != true) {
			digitalWrite(laserPins[channel], LOW);
			laserPinActionStates[channel] = true; // Update state
		}
	}
}

/*
TC1:TCACTUALTEMP?@1		channel:0
TC1:TCACTTEMP?@2			channel:1
TC1:TCACTUALTEMP?@3		channel:2
TC1:TCACTUALTEMP?@4		channel:3
TC1:TCACTUALTEMP?@5		channel:4
TC2:TCACTUALTEMP?@5		channel:5
 */
float readTemperature(int channel) {
	return tempCurrentPoints[channel];
}

/*
TC1:TCOTPHT?@1
TC1:TCOTPHT?@2
TC1:TCOTPHT?@3
TC1:TCOTPHT?@4
TC1:TCOTPHT?@5
TC2:TCOTPHT?@5
 */
float readHighTempSetPoint(int channel) {
	return highTempCurrentPoints[channel];
}

/*
TC1:TCACTUALVOLTAGE?@1
TC1:TCACTVOL?@2
TC1:TCACTUALVOLTAGE?@3
TC1:TCACTUALVOLTAGE?@4
TC1:TCACTUALVOLTAGE?@5
TC2:TCACTUALVOLTAGE?@5
*/
float readTECVoltage(int channel) {
	return voltageCurrentPoints[channel];
}

/*
0
0
0
0
TC1:TCACTCUR?@5
TC2:TCACTCUR?@5
*/
float readTECCurrent(int channel) {
	return currentCurrentPoints[channel];
}

/*
	address: 1~5
	module_index: 1~2
	return: 0~5
 */
uint8_t getChannelIndex(uint8_t address, uint8_t module_index) {
	if (address < 5 && address > 0)
		return address - 1;
	else if(address == 5) {
    if (module_index == 1)
      return 4;
    else if (module_index == 2)
      return 5;
	}
  return ERR_OUT_OF_RANGE;
}

/*
	channel: 0~5
*/
void getAddressModuleFromChannel(uint8_t channel, uint8_t* address, uint8_t* module_index) {
	if (channel < 4 && channel >= 0) {
		*address = channel + 1;
		*module_index = 1;
	}
	else {
		if (channel == 4) {
			*address = 5;
			*module_index = 1;
		}
		else if (channel == 5) {
			*address = 5;
			*module_index = 2;
		}
		else {
			*address = ERR_OUT_OF_RANGE; 
			*module_index = ERR_OUT_OF_RANGE;
		}
	}
}

/*
	Prepare TCM reply state for the next expected response.
	Called after sending a command to Serial5.
	If replyTitle is NULL, the caller must have already set tcm_reply_title
	via sprintf before calling this function.
 */
void tcmPrepareReply(const char* replyTitle, uint8_t address, uint8_t module_index, CommandType cmdType) {
	if (replyTitle != NULL) {
		sprintf(tcm_reply_title, "%s", replyTitle);
	}
	tcm_reply_title_length = strlen(tcm_reply_title);
	tcm_reply_address = address;
	tcm_reply_module = module_index;
	tcm_reply_command_type = cmdType;
	tcm_reply_buf_length = 0;
	reply_frame_analyzing_flag = true;
}

/*
	channel: 0~5
 */
void tcmSwitchTCMStatusCommand(uint8_t channel_index) {
  if (channelStates[channel_index] == PREPARE_SLEEP) {
		// TCSW=0 disable the TCM temperature controller
    tcmDisableEnableTCMCommand(channel_index, false);
  }
  else if (channelStates[channel_index] == WAKE_UP) {
		// TCSW=1 enable the TCM temperature controller
    tcmDisableEnableTCMCommand(channel_index, true);
  }
}

/*
	channel: 0~5
  flag:  True: enable TCM; False: disable TCM
 */
void tcmDisableEnableTCMCommand(uint8_t channel_index, bool flag) {
	uint8_t address = 1; uint8_t module_index = 1;
	getAddressModuleFromChannel(channel_index, &address, &module_index);
	if (address == ERR_OUT_OF_RANGE && module_index == ERR_OUT_OF_RANGE)
		return;

	tcm_command_buf_length = 0;

  if (flag == false)
    sprintf(tcm_command_buf, "TC%d:TCSW=0@%d%c", module_index, address, 0x0D);
  else
    sprintf(tcm_command_buf, "TC%d:TCSW=1@%d%c", module_index, address, 0x0D);

	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));
	tcmPrepareReply("CMD:REPLY=", address, module_index, SWITCHTCMSTATUS);
}

/*
	channel: 0~5
 */
void tcmStartupEnableTCMCommand(uint8_t channel_index) {
	uint8_t address = 1; uint8_t module_index = 1;
	getAddressModuleFromChannel(channel_index, &address, &module_index);
	if (address == ERR_OUT_OF_RANGE && module_index == ERR_OUT_OF_RANGE)
		return;

	tcm_command_buf_length = 0;
	sprintf(tcm_command_buf, "TC%d:TCSW=1@%d%c", module_index, address, 0x0D);
	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));
	tcmPrepareReply("CMD:REPLY=", address, module_index, ENABLETCM);
}

/*
	channel_index: 0~5

	address: 1~5
	module_index: 1~2
 */
void tcmQueryTemperatureCommand(uint8_t channel_index) {
	uint8_t address = 1; uint8_t module_index = 1;
	getAddressModuleFromChannel(channel_index, &address, &module_index);
	if (address == ERR_OUT_OF_RANGE && module_index == ERR_OUT_OF_RANGE)
		return;

	tcm_command_buf_length = 0;

	if (address == 2) {
		sprintf(tcm_command_buf, "TC%d:TCACTTEMP?@%d%c", module_index, address, 0x0D);
	}
	else {
		sprintf(tcm_command_buf, "TC%d:TCACTUALTEMP?@%d%c", module_index, address, 0x0D);
	}
	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));

	if (address == 2)
		sprintf(tcm_reply_title, "TC%d:TCACTTEMP=", module_index);
	else
		sprintf(tcm_reply_title, "TC%d:TCACTUALTEMP=", module_index);

	tcmPrepareReply(NULL, address, module_index, TEMPERATURE);
}

/*
	channel_index: 0~5
 */
void tcmQueryHiTempSetPointCommand(uint8_t channel_index) {
	uint8_t address = 1; uint8_t module_index = 1;
	getAddressModuleFromChannel(channel_index, &address, &module_index);
	if (address == ERR_OUT_OF_RANGE && module_index == ERR_OUT_OF_RANGE)
		return;

	tcm_command_buf_length = 0;
	sprintf(tcm_command_buf, "TC%d:TCOTPHT?@%d%c", module_index, address, 0x0D);
	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));
	sprintf(tcm_reply_title, "TC%d:TCOTPHT=", module_index);
	tcmPrepareReply(NULL, address, module_index, HITEMPSETPOINT);
}

/*
	channel_index: 0~5

	address: 1~5
	module_index: 1~2
 */
void tcmQueryVoltageCommand(uint8_t channel_index) {
	uint8_t address = 1; uint8_t module_index = 1;
	getAddressModuleFromChannel(channel_index, &address, &module_index);
	if (address == ERR_OUT_OF_RANGE && module_index == ERR_OUT_OF_RANGE)
		return;

	tcm_command_buf_length = 0;

	if (address == 2) {
		sprintf(tcm_command_buf, "TC%d:TCACTVOL?@%d%c", module_index, address, 0x0D);
	}
	else {
		sprintf(tcm_command_buf, "TC%d:TCACTUALVOLTAGE?@%d%c", module_index, address, 0x0D);
	}
	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));

	if (address == 2)
		sprintf(tcm_reply_title, "TC%d:TCACTVOL=", module_index);
	else
		sprintf(tcm_reply_title, "TC%d:TCACTUALVOLTAGE=", module_index);

	tcmPrepareReply(NULL, address, module_index, VOLTAGE);
}

/*
	channel_index: 0~5

	address: 1~5
	module_index: 1~2
 */
void tcmQueryCurrentCommand(uint8_t channel_index) {
	uint8_t address = 1; uint8_t module_index = 1;
	getAddressModuleFromChannel(channel_index, &address, &module_index);
	if (address == ERR_OUT_OF_RANGE && module_index == ERR_OUT_OF_RANGE)
		return;

	tcm_command_buf_length = 0;

	sprintf(tcm_command_buf, "TC%d:TCACTCUR?@%d%c", module_index, address, 0x0D);

	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));

	sprintf(tcm_reply_title, "TC%d:TCACTCUR=", module_index);
	tcmPrepareReply(NULL, address, module_index, CURRENT);
}

/*
	channel_index: 0~5

	address: 1~5
	module_index: 1~2
 */
void tcmQueryAdjustTempCommand(uint8_t channel_index) {
	uint8_t address = 1; uint8_t module_index = 1;
	getAddressModuleFromChannel(channel_index, &address, &module_index);
	if (address == ERR_OUT_OF_RANGE && module_index == ERR_OUT_OF_RANGE)
		return;

	tcm_command_buf_length = 0;

	if (address == 2 || address == 5) {
		sprintf(tcm_command_buf, "TC%d:TCADJTEMP?@%d%c", module_index, address, 0x0D);
	}
	else {
		sprintf(tcm_command_buf, "TC%d:TCADJUSTTEMP?@%d%c", module_index, address, 0x0D);
	}

	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));

	if (address == 2 || address == 5)
		sprintf(tcm_reply_title, "TC%d:TCADJTEMP=", module_index);
	else
		sprintf(tcm_reply_title, "TC%d:TCADJUSTTEMP=", module_index);

	tcmPrepareReply(NULL, address, module_index, ADJUSTTEMP);
}

/*
	upload data to upstream host, for example PC
 */
void uploadData(uint8_t* data, uint8_t length)
{
	Serial.write(data, length);
	// flag indicate one frame is over
	uint8_t end[2] = {0x0A, 0x0D};
	Serial.write(end, 2);
}

void sendACK() {
  uint8_t ack = 'A';
  uint32_t ackCRC = crc.calculate(&ack, 1);
  uint8_t ackPacket[5];
  ackPacket[0] = ack;
  memcpy(ackPacket + 1, &ackCRC, 4);
	uploadData(ackPacket, 5);
}

void sendNAK() {
  uint8_t nak = 'N';
  uint32_t nakCRC = crc.calculate(&nak, 1);
  uint8_t nakPacket[5];
  nakPacket[0] = nak;
  memcpy(nakPacket + 1, &nakCRC, 4);
	uploadData(nakPacket, 5);
}

/*
 * query laser channels status shortly
 */
void queryLaserStatus() {
  uint8_t tmp = 0;

	// each 200ms to query laser status 
	if (timeSinceLastQueryLaserStatus < EACH_QUERY_DISTANCE)
		return;
	timeSinceLastQueryLaserStatus = 0;

  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    tmp = digitalRead(laserStatuspins[i]);
    if (laserStatus[i] != tmp) {
      laserStatus[i] = tmp;
      lastLaserStatusChangeTime[i] = millis();
    }
  }

  // also record the key status
  key_status = digitalRead(Interlock_pin);
}

void sendStatus() {
  uint8_t statusPacket[1 + NUM_LASER_CHANNELS + NUM_TEMP_CHANNELS * 7 + NUM_TEMP_CHANNELS * 2 + NUM_TEMP_CHANNELS * 2];
  statusPacket[0] = 'S'; // Status packet identifier

  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    statusPacket[i + 1] = laserStatus[i];
  }

  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    int offset = 1 + NUM_LASER_CHANNELS + i * 7;
    statusPacket[offset] = channelStates[i];
    
    // Convert temperature to 2-byte representation
    int16_t temp = (int16_t)(readTemperature(i) * 100); // Temperature in centidegrees
    statusPacket[offset + 1] = temp >> 8;
    statusPacket[offset + 2] = temp & 0xFF;
    
		temp = (int16_t)(readTECVoltage(i) * 100);
    statusPacket[offset + 3] = temp >> 8;
    statusPacket[offset + 4] = temp & 0xFF;

		temp = (int16_t)(readTECCurrent(i) * 100);
    statusPacket[offset + 5] = temp >> 8;
    statusPacket[offset + 6] = temp & 0xFF;
  }

  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    int offset = 1 + NUM_LASER_CHANNELS + NUM_TEMP_CHANNELS * 7 + i * 2;

    int16_t temp = (int16_t)((readTemperature(i) - tempSetpoints[i]) * 100); // Temperature setpoints
    statusPacket[offset + 0] = temp >> 8;
    statusPacket[offset + 1] = temp & 0xFF;
	}

  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    int offset = 1 + NUM_LASER_CHANNELS + NUM_TEMP_CHANNELS * 7 + NUM_TEMP_CHANNELS * 2 + i * 2;

    int16_t temp = (int16_t)(readHighTempSetPoint(i) * 100); // High Temperature setpoints
    statusPacket[offset + 0] = temp >> 8;
    statusPacket[offset + 1] = temp & 0xFF;
	}

  uint32_t packetCRC = crc.calculate(statusPacket, sizeof(statusPacket));
  uint8_t finalPacket[sizeof(statusPacket) + 4];
  memcpy(finalPacket, statusPacket, sizeof(statusPacket));
  memcpy(finalPacket + sizeof(statusPacket), &packetCRC, 4);

	uploadData(finalPacket, sizeof(finalPacket));
}

/*
 * channel: from 0 to 4
 */
void sendChannelStatus(uint32_t channel) {
  uint8_t statusPacket[1 + 1 + 1];
  statusPacket[0] = 'G'; // Status packet identifier
  statusPacket[1] = channel;
  statusPacket[2] = laserStatus[channel];

  uint32_t packetCRC = crc.calculate(statusPacket, sizeof(statusPacket));
  uint8_t finalPacket[sizeof(statusPacket) + 4];
  memcpy(finalPacket, statusPacket, sizeof(statusPacket));
  memcpy(finalPacket + sizeof(statusPacket), &packetCRC, 4);

  uploadData(finalPacket, sizeof(finalPacket));
}

/*
	Read a 4-byte little-endian channel number from buffer[1..4].
	Caller must ensure buffer has at least 5 bytes (1 command byte + 4 data bytes).
 */
uint32_t readChannelFromBuffer(const uint8_t* buffer) {
  return uint32_t(buffer[1]) | (uint32_t(buffer[2]) << 8) |
         (uint32_t(buffer[3]) << 16) | (uint32_t(buffer[4]) << 24);
}

void onPacketReceived(const uint8_t* buffer, size_t size) {
  if (size < 5) return; // Minimum packet size (1 byte command + 4 bytes CRC)

  uint32_t receivedCRC;
  memcpy(&receivedCRC, buffer + size - 4, 4);
  uint32_t calculatedCRC = crc.calculate(buffer, size - 4);

  if (receivedCRC != calculatedCRC) {
    sendNAK();
    return;
  }

  switch (buffer[0]) {
    case 'Q': // Query status
      sendStatus();
      break;
    case 'P': // Set parameters
      setParameters(buffer, size - 4);
      break;
    case 'S': // Put one channel to sleep
      {
        if (size < 9) { sendNAK(); break; } // 1 cmd + 4 channel + 4 CRC
        uint32_t channel = readChannelFromBuffer(buffer);
        if (channel >= NUM_LASER_CHANNELS) {
          sendNAK();
          break;
        }
        if (channel == 4) {
          doSleepAction(4);
          doSleepAction(5);
        }
        else
          doSleepAction(channel);
      }
      break;
    case 'W': // Wake up channel
      {
        if (size < 9) { sendNAK(); break; } // 1 cmd + 4 channel + 4 CRC
        uint32_t channel = readChannelFromBuffer(buffer);
        if (channel >= NUM_LASER_CHANNELS) {
          sendNAK();
          break;
        }
        if (channel == 4) {
          doWakeupAction(4);
          doWakeupAction(5);
        }
        else
          doWakeupAction(channel);
      }
      break;
    case 'G': // Query channel status
      {
        if (size < 9) { sendNAK(); break; } // 1 cmd + 4 channel + 4 CRC
        uint32_t channel = readChannelFromBuffer(buffer);
        if (channel >= NUM_LASER_CHANNELS) {
          sendNAK();
          break;
        }
        sendChannelStatus(channel);
      }
      break;
  }
}

/*
	get a char position from a char array 

	return: offset address of the char, if not found return -1 
				
 */
int getCharOffset(char* arrayValue, char searchChar) {
	char *p = strchr(arrayValue, searchChar);
	if (p != NULL) {
		return p - arrayValue;
	}
	else {
		return -1; 
	}
}

/*
	convert char array to float	

 */
float convertChararrayToFloat(char * arrayValue, int valueLength) {
	char inArray[valueLength + 1]	= {0};
	for (int i = 0; i < valueLength; i++) {
		inArray[i] = arrayValue[i];
	}

	String value(inArray);
	return value.toFloat();
}

/*
	get information from one frame reply

	return: true, the retvalue is the available value
				false, the retvalue is the unavailable value
 */
bool analyzeValueFromTCMProtocol(float *retvalue) {
	if (strncmp(tcm_reply_title, tcm_reply_buf, tcm_reply_title_length) == 0) {
		int offset = getCharOffset(&tcm_reply_buf[tcm_reply_title_length], '@');
		if (offset == -1)
			return false;

		*retvalue = convertChararrayToFloat(&tcm_reply_buf[tcm_reply_title_length], offset);
		return true;
	}
	else
		return false;
}

/*
	query loop, call it in the main loop
*/
void queryTCMDataMainLoop() {
	// each 200ms to query one channel data from TMC module
	if (timeSinceLastQueryTemp < EACH_QUERY_DISTANCE)
		return;
	timeSinceLastQueryTemp = 0;
	
	switch (query_frequnce[current_query_type_index++]) {
		case TEMPERATURE:
			{
				// send the query frame to one of TCM modules
				tcmQueryTemperatureCommand(gQueryTemperatureChannelIndex++);
				if (gQueryTemperatureChannelIndex == NUM_TEMP_CHANNELS)
					gQueryTemperatureChannelIndex = 0;
			}
			break;
		case VOLTAGE: 
			{
				// send the query frame to one of TCM modules
				tcmQueryVoltageCommand(gQueryVoltageChannelIndex++);
				if (gQueryVoltageChannelIndex == NUM_VOLTAGE_CHANNELS)
					gQueryVoltageChannelIndex = 0;
			}
			break;
		case CURRENT:
			{
				// only module 5's two current need to be query
				if (gQueryCurrentChannelIndex < 4)
					gQueryCurrentChannelIndex = 4;

				// send the query frame to one of TCM modules
				tcmQueryCurrentCommand(gQueryCurrentChannelIndex++);
				if (gQueryCurrentChannelIndex == NUM_CURRENT_CHANNELS)
					gQueryCurrentChannelIndex = 0;
			}
			break;
		case HITEMPSETPOINT:
			{
				// send the query frame to one of TCM modules
				tcmQueryHiTempSetPointCommand(gQueryHiTempSetPointChannelIndex++);
				if (gQueryHiTempSetPointChannelIndex == NUM_TEMP_CHANNELS)
					gQueryHiTempSetPointChannelIndex = 0;
			}
			break;
		case SWITCHTCMSTATUS:
			{
				tcmSwitchTCMStatusCommand(gProcessChannelStatusIndex++);
				if (gProcessChannelStatusIndex == NUM_TEMP_CHANNELS)
					gProcessChannelStatusIndex = 0;
			}
			break;
		default:
			break;
	}

	if (current_query_type_index == MAX_QUERY_TYPE_INDEX)
		current_query_type_index = 0;
}

/*
	analyzing host(such as PC) frame
 */
void analyzingHostFrame() {
	if (Serial.available()) {
		if (host_protocol_buf_length >= sizeof(host_protocol_buf)) {
			Serial.read(); // drain the byte
			host_protocol_buf_length = 0;
			return;
		}
		host_protocol_buf[host_protocol_buf_length++] = Serial.read();
		if (host_protocol_buf_length >= 2 &&
				host_protocol_buf[host_protocol_buf_length - 1] == 0x0D &&
				host_protocol_buf[host_protocol_buf_length - 2] == 0x0A) {
			onPacketReceived(host_protocol_buf, host_protocol_buf_length - 2);

			host_protocol_buf_length = 0;
		}
	}
}

/*
 * nomatter what, enable all channel TCMs when power up
 */
void enableAllTCMs() {
  timeSinceLastQueryTemp = 0;

  while(enableTCMReady[NUM_TEMP_CHANNELS - 1] == false) {
    // each 200ms to query one channel data from TMC module
    if (timeSinceLastQueryTemp >= EACH_QUERY_DISTANCE) {
      timeSinceLastQueryTemp = 0;

      tcmStartupEnableTCMCommand(gEnableTCMChannelIndex);
    }

    // TCM modules protocol process
    analyzingTCMFrame();
  }
}

/*
 get adjust temperature value from TCM, before enter main loop
 */
void getAdjustTemperature() {
  timeSinceLastQueryTemp = 0;

  while(getTempReady[NUM_TEMP_CHANNELS - 1] == false) {
    // each 200ms to query one channel data from TMC module
    if (timeSinceLastQueryTemp >= EACH_QUERY_DISTANCE) {
      timeSinceLastQueryTemp = 0;

      // send the query frame to one of TCM modules
      tcmQueryAdjustTempCommand(gQueryAdjustTempChannelIndex);
    }

    // TCM modules protocol process
    analyzingTCMFrame();
  }
}

/*
	Try to parse a TCM reply value and store it in the target array.
	Returns the channel index on success, or ERR_OUT_OF_RANGE on failure.
 */
uint8_t tcmParseAndStore(float* targetArray) {
	float tvalue = 0;
	if (analyzeValueFromTCMProtocol(&tvalue)) {
		uint8_t tindex = getChannelIndex(tcm_reply_address, tcm_reply_module);
		if (tindex != ERR_OUT_OF_RANGE) {
			targetArray[tindex] = tvalue;
		}
		return tindex;
	}
	tcm_parse_failure_count++;
	return ERR_OUT_OF_RANGE;
}

/*
	Analyze TCM modules frame
 */
void analyzingTCMFrame() {
	if (reply_frame_analyzing_flag) {
		if (Serial5.available()) {
			if (tcm_reply_buf_length >= sizeof(tcm_reply_buf)) {
				Serial5.read(); // drain
				tcm_reply_buf_length = 0;
				reply_frame_analyzing_flag = false;
				tcm_parse_failure_count++;
				return;
			}
			tcm_reply_buf[tcm_reply_buf_length++] = Serial5.read();
			// read the end flag of frame
			if (tcm_reply_buf[tcm_reply_buf_length - 1] == 0x0D) {

				switch (tcm_reply_command_type) {
					case TEMPERATURE:
						tcmParseAndStore(tempCurrentPoints);
						break;
					case VOLTAGE:
						tcmParseAndStore(voltageCurrentPoints);
						break;
					case CURRENT:
						tcmParseAndStore(currentCurrentPoints);
						break;
					case HITEMPSETPOINT:
						tcmParseAndStore(highTempCurrentPoints);
						break;
					case ADJUSTTEMP:
						{
							uint8_t tindex = tcmParseAndStore(tempSetpoints);
							if (tindex != ERR_OUT_OF_RANGE) {
								getTempReady[tindex] = true;
								gQueryAdjustTempChannelIndex++;
								if (gQueryAdjustTempChannelIndex == NUM_TEMP_CHANNELS)
									gQueryAdjustTempChannelIndex = 0;
							}
						}
						break;
          case ENABLETCM:
            {
              float tvalue = 0;
              if (analyzeValueFromTCMProtocol(&tvalue)) {
                uint8_t tindex = getChannelIndex(tcm_reply_address, tcm_reply_module);
                if (tindex != ERR_OUT_OF_RANGE) {
                  if (int(tvalue) == 1) {
                    enableTCMReady[tindex] = true;
                    // only be successful could deal with next channel
                    gEnableTCMChannelIndex ++;
                    if (gEnableTCMChannelIndex == NUM_TEMP_CHANNELS)
                      gEnableTCMChannelIndex = 0;
                  }
                }
              }
            }
						break;
					case SWITCHTCMSTATUS:
						{
							float tvalue = 0;
							if (analyzeValueFromTCMProtocol(&tvalue)) {
								uint8_t tindex = getChannelIndex(tcm_reply_address, tcm_reply_module);
                if (tindex != ERR_OUT_OF_RANGE) {
                  if (int(tvalue) == 1) {
                    if (channelStates[tindex] == PREPARE_SLEEP)
                      updateChannelStatus(tindex, SLEEP);
                    else if(channelStates[tindex] == WAKE_UP)
                      updateChannelStatus(tindex, WARMING_UP);
                  }
                }
							}
						}
						break;
					default:
						break;
				}

				// finish frame analyzing, reset flag
				reply_frame_analyzing_flag = false;
				tcm_reply_buf_length = 0;
				tcm_reply_address = 0;
				tcm_reply_module = 0;
			}
		}
	}
}

/*
 * when Channel status change, update the value and the change time
 */
void updateChannelStatus(int channel, ChannelState status) {
  if (channelStates[channel] != status)
    lastChannelStatesChangeTime[channel] = millis();
  // means the channel realy enter to SLEEP status
  channelStates[channel] = status;
}

/*
 * channel_index: 0~5
 */
unsigned long getLaserStatusChangeTime(int channel_index) {
	// from 0 ~ 3
	if (channel_index < (NUM_LASER_CHANNELS - 1))
		return lastLaserStatusChangeTime[channel_index];

	// 4, 5 return last laser channel value
	return lastLaserStatusChangeTime[NUM_LASER_CHANNELS - 1];
}

/*
 * channel_index: 0~5
 */
uint8_t getLaserStatus(int channel_index) {
	// from 0 ~ 3
	if (channel_index < (NUM_LASER_CHANNELS - 1))
		return laserStatus[channel_index];

	// 4, 5 return last laser channel value
	return laserStatus[NUM_LASER_CHANNELS - 1];
}

void setup() {
  Serial.begin(115200);
	delayMicroseconds(100000);

  // TCM104x module UART5
  Serial5.begin(57600);
	delayMicroseconds(100000);	

  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, LOW);

  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, LOW);

  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, LOW);

  digitalWrite(LED_G, HIGH);

  // set the control pins to OUTPUT status
  // and disable the control pins as well
  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    pinMode(laserPins[i], OUTPUT);
    digitalWrite(laserPins[i], HIGH);
  }

  pinMode(Dispecker_pin, OUTPUT);
  digitalWrite(Dispecker_pin, HIGH);
	
	// laser port start pins initialize
  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    pinMode(laserStatuspins[i], INPUT);
  }

  pinMode(Interlock_pin, INPUT);

  // initialize the last laser status change time
  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    lastLaserStatusChangeTime[i] = millis();
  }

  // before enter main loop, get adjust temperature from TCMs
  getAdjustTemperature();

  // enable all channels TCM
  // sometimes application would disable TCM before it power off 
  // so need be enable all channels TCM
  enableAllTCMs();

	timeSinceLastQueryTemp = 0;

  // Initialize the last transition
  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    lastChannelStatesChangeTime[i] = millis();
  }
  
  // only the channels status reach ACTIVE status could enable laser
  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    disableLaser(i);
  }
}

// do real sleep action
void doSleepAction(int i) {
  disableLaser(i);
  updateChannelStatus(i, PREPARE_SLEEP);
  recordSleepTime[i] = millis();
  wakeupCoolDownTime[i] = SHORT_WAKEUP_COOLDOWN_TIME;
}

void doWakeupAction(int i) {
  // do not need enable laser immediately.
  // when channel status enter ACTIVE, then enable laser
  //enableLaser(i);
  updateChannelStatus(i, WAKE_UP);

  // enforce to reset laserStatus ChangeTime
  // laser chanel is 5 and status chanel is 6
  if (i == 5)
    i = 4;

  // enforce to reset laserStatus ChangeTime
  // otherwise the application would fall into sleep
  // again
  laserStatus[i] = digitalRead(laserStatuspins[i]);
  lastLaserStatusChangeTime[i] = millis();
}

void doEnableLasersAction() {
  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    if (i != 4 && i != 5) {
      if (channelStates[i] == ACTIVE) {
        enableLaser(i);
      }
    }
    else {
      if (channelStates[4] == ACTIVE && channelStates[5] == ACTIVE) {
        enableLaser(i);
      }
    }
  }
}

void loop() {
  // query laser status
  queryLaserStatus();

	// query TCM module data
	queryTCMDataMainLoop();

  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    float currentTemp = readTemperature(i);
    float tempDiff = currentTemp - tempSetpoints[i];

    switch (channelStates[i]) {
      case WARMING_UP:
        if (abs(tempDiff) <= 0.5) {
          updateChannelStatus(i, CHECK_ACTIVE);
        }
				else if (tempDiff > TEMP_ERROR_THRESHOLD) {
          updateChannelStatus(i, CHECK_ERROR);
				}
        break;
      case CHECK_ACTIVE:
        if (abs(tempDiff) > 0.5) {
					if (tempDiff > TEMP_ERROR_THRESHOLD) {
          	updateChannelStatus(i, CHECK_ERROR);
					}
					else {
          	updateChannelStatus(i, WARMING_UP);
					}
        }
        else {
          if ((millis() - lastChannelStatesChangeTime[i]) >= ACTIVE_DURATION) {
            //enableLaser(i);
            updateChannelStatus(i, ACTIVE);
          }
        }
        break;
      case ACTIVE:
        if (tempDiff >= TEMP_ERROR_THRESHOLD) {
          updateChannelStatus(i, CHECK_ERROR);
        }
        else {
          if (tempDiff <= -0.5) {
            updateChannelStatus(i, WARMING_UP);
          }
        }
        break;
      case CHECK_ERROR:
        if (tempDiff < TEMP_ERROR_THRESHOLD) {
          if ( tempDiff >= -0.5 ) {
            updateChannelStatus(i, ACTIVE);
          }
          else {
            updateChannelStatus(i, WARMING_UP);
          }
        }
        else {
          if ((millis() - lastChannelStatesChangeTime[i]) >= ERROR_DURATION) {
            disableLaser(i);
            updateChannelStatus(i, ERROR);
          }
        }
        break;
      case ERROR:
        if (abs(tempDiff) <= 0.5) {
          updateChannelStatus(i, CHECK_ACTIVE);
        }
        break;
      case SLEEP:
        if (getLaserStatus(i) == 1) {
          // need keep the wakeup signed be over cool down time
          if (millis() - recordSleepTime[i] > wakeupCoolDownTime[i])
            updateChannelStatus(i, WAKE_UP);
        }
        break;
      case WAKE_UP:
        // do nothing, just wait for TCM be changed to enable status
				// then enter WARMING_UP status
        break;
      case PREPARE_SLEEP:
        // do nothing, just wait for TCM be changed to disable status 
				// then enter SLEEP status 
        break;
    }

    if (channelStates[i] == WARMING_UP || channelStates[i] == ACTIVE || channelStates[i] == ERROR) {
      // the laser status is low and kept the status over SLEEP_TIMEOUT, then be ready to SLEEP status
      if (getLaserStatus(i) == 0 && (millis() - getLaserStatusChangeTime(i)) >= CHANNELS_SLEEP_TIMEOUT[i]) {
        doSleepAction(i);
      }
    }
  }

  // only the chanel's status is at ACTIVE
  // we could enable the channel laser 
  // especially for channel 4 and 5
  doEnableLasersAction();

  // indicate the device status used LED color
  // Red: if there is one channel ERROR
  // Gree: all channels are ACTIVE 
  // Blue: others 
  indicate_device_status();

	// analyzing host frame
	analyzingHostFrame();

	// TCM modules protocol process
	analyzingTCMFrame();
}
