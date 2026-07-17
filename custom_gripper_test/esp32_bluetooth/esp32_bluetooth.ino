#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Wire.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled for the selected ESP32 board.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth SPP is unavailable. Select an ESP32 board with Classic Bluetooth support.
#endif

namespace {
constexpr char kBluetoothName[] = "STS3215_Gripper";
constexpr size_t kMaxCommandLength = 96;
constexpr int kServoRxPin = 18;
constexpr int kServoTxPin = 19;
constexpr uint32_t kServoBaudRate = 1000000;
constexpr uint8_t kFirstServoId = 0;
constexpr uint8_t kLastServoId = 20;
constexpr int kServoMiddlePosition = 2047;
constexpr int kServoMinPosition = 0;
constexpr int kServoMaxPosition = 4095;
constexpr int kDefaultMoveSpeed = 200;
constexpr int kDefaultMoveAcceleration = 20;
constexpr int kGripperCommandMin = 0;
constexpr int kGripperCommandMax = 500;
constexpr int kServo1OpenRaw = 2430;
constexpr int kServo1ClosedRaw = 1553;
constexpr int kServo2OpenRaw = 1547;
constexpr int kServo2ClosedRaw = 2424;
constexpr int kConnectMoveSpeed = 100;
constexpr int kConnectMoveAcceleration = 10;
constexpr int kLoadCurrentThreshold = 100;
constexpr int kLoadConsecutiveSamples = 2;
constexpr uint32_t kLoadPollIntervalMs = 50;
constexpr int kTargetPositionTolerance = 8;
constexpr int kLoadLatchReleaseCommandDelta = 20;
constexpr int kOledWidth = 128;
constexpr int kOledHeight = 32;
constexpr int kOledSdaPin = 21;
constexpr int kOledSclPin = 22;
constexpr uint8_t kOledAddress = 0x3c;

BluetoothSerial SerialBT;
String bluetoothCommand;
bool bluetoothStarted = false;
bool displayAvailable = false;
volatile bool bluetoothConnected = false;
volatile bool bluetoothStateChanged = false;
bool gripperHomeRequested = false;
bool loadProtectionActive = false;
int loadTarget1Raw = 0;
int loadTarget2Raw = 0;
int loadHighSampleCount = 0;
uint32_t lastLoadPollMs = 0;
bool loadStopLatched = false;
int loadStopCommand = 0;
int activeGripperCommand = 0;

bool rawPositionIsSafe(int id, int rawPosition) {
  if (id == 1) {
    return rawPosition >= min(kServo1OpenRaw, kServo1ClosedRaw) &&
           rawPosition <= max(kServo1OpenRaw, kServo1ClosedRaw);
  }
  if (id == 2) {
    return rawPosition >= min(kServo2OpenRaw, kServo2ClosedRaw) &&
           rawPosition <= max(kServo2OpenRaw, kServo2ClosedRaw);
  }
  return true;
}

uint8_t oledBuffer[kOledWidth * kOledHeight / 8] = {};

const uint8_t kFont[][5] = {
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, {0x00, 0x42, 0x7f, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4b, 0x31},
    {0x18, 0x14, 0x12, 0x7f, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1e},
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, {0x7f, 0x49, 0x49, 0x49, 0x36},
    {0x3e, 0x41, 0x41, 0x41, 0x22}, {0x7f, 0x41, 0x41, 0x22, 0x1c},
    {0x7f, 0x49, 0x49, 0x49, 0x41}, {0x7f, 0x09, 0x09, 0x09, 0x01},
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, {0x7f, 0x08, 0x08, 0x08, 0x7f},
    {0x00, 0x41, 0x7f, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3f, 0x01},
    {0x7f, 0x08, 0x14, 0x22, 0x41}, {0x7f, 0x40, 0x40, 0x40, 0x40},
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, {0x7f, 0x04, 0x08, 0x10, 0x7f},
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, {0x7f, 0x09, 0x09, 0x09, 0x06},
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, {0x7f, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7f, 0x01, 0x01},
    {0x3f, 0x40, 0x40, 0x40, 0x3f}, {0x1f, 0x20, 0x40, 0x20, 0x1f},
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43},
};

void oledCommand(uint8_t command) {
  Wire.beginTransmission(kOledAddress);
  Wire.write(0x00);
  Wire.write(command);
  Wire.endTransmission();
}

bool initializeDisplay() {
  Wire.beginTransmission(kOledAddress);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  const uint8_t commands[] = {
      0xae, 0xd5, 0x80, 0xa8, 0x1f, 0xd3, 0x00, 0x40, 0x8d, 0x14,
      0x20, 0x00, 0xa1, 0xc8, 0xda, 0x02, 0x81, 0x8f, 0xd9, 0xf1,
      0xdb, 0x40, 0xa4, 0xa6, 0xaf,
  };
  for (uint8_t command : commands) {
    oledCommand(command);
  }
  return true;
}

void updateDisplay() {
  oledCommand(0x21);
  oledCommand(0);
  oledCommand(kOledWidth - 1);
  oledCommand(0x22);
  oledCommand(0);
  oledCommand((kOledHeight / 8) - 1);
  for (size_t offset = 0; offset < sizeof(oledBuffer); offset += 16) {
    Wire.beginTransmission(kOledAddress);
    Wire.write(0x40);
    Wire.write(oledBuffer + offset, 16);
    Wire.endTransmission();
  }
}

const uint8_t *glyphFor(char character) {
  if (character >= '0' && character <= '9') {
    return kFont[character - '0'];
  }
  if (character >= 'a' && character <= 'z') {
    character -= 'a' - 'A';
  }
  if (character >= 'A' && character <= 'Z') {
    return kFont[10 + character - 'A'];
  }
  return nullptr;
}

void drawText(int x, int y, const char *text) {
  while (*text != '\0' && x <= kOledWidth - 5) {
    const uint8_t *glyph = glyphFor(*text++);
    if (glyph != nullptr) {
      for (int column = 0; column < 5; ++column) {
        for (int row = 0; row < 7; ++row) {
          if ((glyph[column] & (1 << row)) != 0) {
            const int pixelY = y + row;
            oledBuffer[(pixelY / 8) * kOledWidth + x + column] |=
                1 << (pixelY % 8);
          }
        }
      }
    }
    x += 6;
  }
}

void showStatus(const char *title, const char *detail = nullptr) {
  if (!displayAvailable) {
    return;
  }
  memset(oledBuffer, 0, sizeof(oledBuffer));
  drawText(0, 2, title);
  if (detail != nullptr) {
    drawText(0, 18, detail);
  }
  updateDisplay();
}

void bluetoothCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *parameter) {
  (void)parameter;
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    bluetoothConnected = true;
    bluetoothStateChanged = true;
  } else if (event == ESP_SPP_CLOSE_EVT) {
    bluetoothConnected = false;
    bluetoothStateChanged = true;
  }
}

class Sts3215Bus {
 public:
  explicit Sts3215Bus(HardwareSerial &serial) : serial_(serial) {}

  int ping(uint8_t id) {
    sendPacket(id, 0x01, nullptr, 0);
    return readStatus(id, nullptr, 0) >= 0 ? id : -1;
  }

  int readPosition(uint8_t id) {
    const uint8_t parameters[] = {56, 2};
    uint8_t response[2];
    sendPacket(id, 0x02, parameters, sizeof(parameters));
    return readStatus(id, response, sizeof(response)) == 2
               ? response[0] | (response[1] << 8)
               : -1;
  }

  int readCurrent(uint8_t id) {
    const uint8_t parameters[] = {69, 2};
    uint8_t response[2];
    sendPacket(id, 0x02, parameters, sizeof(parameters));
    if (readStatus(id, response, sizeof(response)) != 2) {
      return -1;
    }
    return (response[0] | (response[1] << 8)) & 0x7fff;
  }

  void calibrateOffset(uint8_t id) { writeByte(id, 40, 128); }

  void enableTorque(uint8_t id, bool enabled) {
    writeByte(id, 40, enabled ? 1 : 0);
  }

  void changeId(uint8_t currentId, uint8_t newId) {
    writeByte(currentId, 55, 0);
    delay(20);
    writeByte(currentId, 5, newId);
    delay(20);
    writeByte(newId, 55, 1);
    delay(20);
  }

  void clearCalibrationOffset(uint8_t id) {
    writeByte(id, 55, 0);
    delay(20);
    const uint8_t parameters[] = {31, 0, 0};
    sendPacket(id, 0x03, parameters, sizeof(parameters));
    delay(20);
    writeByte(id, 55, 1);
    delay(20);
  }

  void writePosition(
      uint8_t id,
      int16_t position,
      uint16_t speed,
      uint8_t acceleration) {
    const uint8_t parameters[] = {
        41,
        acceleration,
        static_cast<uint8_t>(position & 0xff),
        static_cast<uint8_t>((position >> 8) & 0xff),
        0,
        0,
        static_cast<uint8_t>(speed & 0xff),
        static_cast<uint8_t>((speed >> 8) & 0xff),
    };
    sendPacket(id, 0x03, parameters, sizeof(parameters));
  }

  void syncWriteTwoPositions(
      uint8_t firstId,
      int16_t firstPosition,
      uint8_t secondId,
      int16_t secondPosition,
      uint16_t speed,
      uint8_t acceleration) {
    uint8_t parameters[18] = {41, 7};
    const uint8_t ids[] = {firstId, secondId};
    const int16_t positions[] = {firstPosition, secondPosition};
    for (int servo = 0; servo < 2; ++servo) {
      const int offset = 2 + servo * 8;
      parameters[offset] = ids[servo];
      parameters[offset + 1] = acceleration;
      parameters[offset + 2] = positions[servo] & 0xff;
      parameters[offset + 3] = (positions[servo] >> 8) & 0xff;
      parameters[offset + 4] = 0;
      parameters[offset + 5] = 0;
      parameters[offset + 6] = speed & 0xff;
      parameters[offset + 7] = (speed >> 8) & 0xff;
    }
    sendPacket(0xfe, 0x83, parameters, sizeof(parameters));
  }

 private:
  HardwareSerial &serial_;

  void writeByte(uint8_t id, uint8_t address, uint8_t value) {
    const uint8_t parameters[] = {address, value};
    sendPacket(id, 0x03, parameters, sizeof(parameters));
  }

  void sendPacket(
      uint8_t id,
      uint8_t instruction,
      const uint8_t *parameters,
      size_t parameterCount) {
    while (serial_.available()) {
      serial_.read();
    }

    const uint8_t length = parameterCount + 2;
    uint8_t checksum = id + length + instruction;
    serial_.write(0xff);
    serial_.write(0xff);
    serial_.write(id);
    serial_.write(length);
    serial_.write(instruction);
    for (size_t index = 0; index < parameterCount; ++index) {
      serial_.write(parameters[index]);
      checksum += parameters[index];
    }
    serial_.write(static_cast<uint8_t>(~checksum));
    serial_.flush();
  }

  int readStatus(uint8_t expectedId, uint8_t *parameters, size_t capacity) {
    const uint32_t deadline = millis() + 20;
    bool firstHeaderFound = false;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
      if (!serial_.available()) {
        delay(1);
        continue;
      }
      const uint8_t value = serial_.read();
      if (!firstHeaderFound) {
        firstHeaderFound = value == 0xff;
        continue;
      }
      if (value == 0xff) {
        break;
      }
      firstHeaderFound = false;
    }

    if (!firstHeaderFound || !waitForBytes(3, deadline)) {
      return -1;
    }
    const uint8_t id = serial_.read();
    const uint8_t length = serial_.read();
    const uint8_t error = serial_.read();
    if (id != expectedId || length < 2) {
      return -1;
    }

    const size_t parameterCount = length - 2;
    if (!waitForBytes(parameterCount + 1, deadline)) {
      return -1;
    }
    uint8_t checksum = id + length + error;
    for (size_t index = 0; index < parameterCount; ++index) {
      const uint8_t value = serial_.read();
      checksum += value;
      if (index < capacity && parameters != nullptr) {
        parameters[index] = value;
      }
    }
    const uint8_t receivedChecksum = serial_.read();
    if (receivedChecksum != static_cast<uint8_t>(~checksum) || error != 0 ||
        parameterCount > capacity) {
      return -1;
    }
    return parameterCount;
  }

  bool waitForBytes(size_t count, uint32_t deadline) {
    while (serial_.available() < static_cast<int>(count)) {
      if (static_cast<int32_t>(deadline - millis()) <= 0) {
        return false;
      }
      delay(1);
    }
    return true;
  }
};

Sts3215Bus servoBus(Serial1);

void reply(const String &message) {
  Serial.println(message);
  if (bluetoothStarted) {
    SerialBT.println(message);
  }
}

uint8_t calibrateConnectedServos() {
  reply("Scanning servo IDs 0..20 for zero calibration");
  uint8_t foundCount = 0;

  for (uint16_t id = kFirstServoId; id <= kLastServoId; ++id) {
    if (servoBus.ping(static_cast<uint8_t>(id)) == -1) {
      continue;
    }

    ++foundCount;
    reply("Servo found: ID " + String(id));
    servoBus.calibrateOffset(static_cast<uint8_t>(id));
    delay(50);

    const int position = servoBus.readPosition(static_cast<uint8_t>(id));
    reply(
        "Zero calibrated: ID " + String(id) +
        ", raw=" + String(position) + ", logical=0");
  }

  if (foundCount == 0) {
    reply("ERROR: no servo found");
    return 0;
  }

  reply("Zero calibration complete: " + String(foundCount) + " servo(s)");
  return foundCount;
}

bool parseId(const String &text, int &id) {
  char trailing = '\0';
  return sscanf(text.c_str(), "%d %c", &id, &trailing) == 1 &&
         id >= kFirstServoId && id <= 253;
}

void readServoPosition(const String &arguments) {
  int id = -1;
  if (!parseId(arguments, id)) {
    reply("ERROR: use READ <id>");
    return;
  }

  const int rawPosition = servoBus.readPosition(static_cast<uint8_t>(id));
  if (rawPosition == -1) {
    reply("ERROR: servo not found: ID " + String(id));
    return;
  }

  reply(
      "POSITION id=" + String(id) + " logical=" +
      String(rawPosition - kServoMiddlePosition) + " raw=" +
      String(rawPosition));
}

void moveServo(const String &arguments) {
  int id = -1;
  int logicalPosition = 0;
  int speed = kDefaultMoveSpeed;
  int acceleration = kDefaultMoveAcceleration;
  const int parsed = sscanf(
      arguments.c_str(),
      "%d %d %d %d",
      &id,
      &logicalPosition,
      &speed,
      &acceleration);

  if (parsed < 2 || id < kFirstServoId || id > 253) {
    reply("ERROR: use MOVE <id> <logical> [speed] [acc]");
    return;
  }

  const int rawPosition = logicalPosition + kServoMiddlePosition;
  if (rawPosition < kServoMinPosition || rawPosition > kServoMaxPosition ||
      speed < 1 || speed > 3073 || acceleration < 0 || acceleration > 150) {
    reply("ERROR: logical=-2047..2048 speed=1..3073 acc=0..150");
    return;
  }
  if (!rawPositionIsSafe(id, rawPosition)) {
    reply("ERROR: target exceeds configured gripper limits");
    return;
  }

  if (servoBus.ping(static_cast<uint8_t>(id)) == -1) {
    reply("ERROR: servo not found: ID " + String(id));
    return;
  }

  servoBus.writePosition(
      static_cast<uint8_t>(id),
      static_cast<int16_t>(rawPosition),
      static_cast<uint16_t>(speed),
      static_cast<uint8_t>(acceleration));
  reply(
      "MOVE sent: id=" + String(id) + " logical=" +
      String(logicalPosition) + " raw=" + String(rawPosition));
}

void moveServoRaw(const String &arguments) {
  int id = -1;
  int rawPosition = 0;
  int speed = kDefaultMoveSpeed;
  int acceleration = kDefaultMoveAcceleration;
  const int parsed = sscanf(
      arguments.c_str(), "%d %d %d %d", &id, &rawPosition, &speed, &acceleration);
  if (parsed < 2 || id < 0 || id > 253) {
    reply("ERROR: use RAW_MOVE <id> <raw> [speed] [acc]");
    return;
  }
  if (rawPosition < kServoMinPosition || rawPosition > kServoMaxPosition ||
      speed < 1 || speed > 3073 || acceleration < 0 || acceleration > 150) {
    reply("ERROR: raw=0..4095 speed=1..3073 acc=0..150");
    return;
  }
  if (!rawPositionIsSafe(id, rawPosition)) {
    reply("ERROR: raw target exceeds configured gripper limits");
    return;
  }
  if (servoBus.ping(static_cast<uint8_t>(id)) == -1) {
    reply("ERROR: servo not found: ID " + String(id));
    return;
  }
  servoBus.writePosition(
      static_cast<uint8_t>(id),
      static_cast<int16_t>(rawPosition),
      static_cast<uint16_t>(speed),
      static_cast<uint8_t>(acceleration));
  reply("RAW_MOVE sent: id=" + String(id) + " raw=" + String(rawPosition));
}

void syncMoveServos(const String &arguments) {
  int firstId = -1;
  int firstLogical = 0;
  int secondId = -1;
  int secondLogical = 0;
  int speed = kDefaultMoveSpeed;
  int acceleration = kDefaultMoveAcceleration;
  const int parsed = sscanf(
      arguments.c_str(),
      "%d %d %d %d %d %d",
      &firstId,
      &firstLogical,
      &secondId,
      &secondLogical,
      &speed,
      &acceleration);

  if (parsed < 4 || firstId < 0 || firstId > 253 || secondId < 0 ||
      secondId > 253 || firstId == secondId) {
    reply("ERROR: use SYNC_MOVE <id1> <pos1> <id2> <pos2> [speed] [acc]");
    return;
  }
  const int firstRaw = firstLogical + kServoMiddlePosition;
  const int secondRaw = secondLogical + kServoMiddlePosition;
  if (firstRaw < kServoMinPosition || firstRaw > kServoMaxPosition ||
      secondRaw < kServoMinPosition || secondRaw > kServoMaxPosition ||
      speed < 1 || speed > 3073 || acceleration < 0 || acceleration > 150) {
    reply("ERROR: logical=-2047..2048 speed=1..3073 acc=0..150");
    return;
  }
  if (!rawPositionIsSafe(firstId, firstRaw) ||
      !rawPositionIsSafe(secondId, secondRaw)) {
    reply("ERROR: target exceeds configured gripper limits");
    return;
  }
  if (servoBus.ping(static_cast<uint8_t>(firstId)) == -1 ||
      servoBus.ping(static_cast<uint8_t>(secondId)) == -1) {
    reply("ERROR: one or both servo IDs were not found");
    return;
  }

  servoBus.syncWriteTwoPositions(
      static_cast<uint8_t>(firstId),
      static_cast<int16_t>(firstRaw),
      static_cast<uint8_t>(secondId),
      static_cast<int16_t>(secondRaw),
      static_cast<uint16_t>(speed),
      static_cast<uint8_t>(acceleration));
  reply(
      "SYNC_MOVE sent: id=" + String(firstId) + " logical=" +
      String(firstLogical) + ", id=" + String(secondId) + " logical=" +
      String(secondLogical));
}

void pairMoveServos(const String &arguments) {
  int logicalPosition = 0;
  int speed = kDefaultMoveSpeed;
  int acceleration = kDefaultMoveAcceleration;
  const int parsed = sscanf(
      arguments.c_str(), "%d %d %d", &logicalPosition, &speed, &acceleration);
  if (parsed < 1) {
    reply("ERROR: use PAIR_MOVE <position> [speed] [acc]");
    return;
  }
  if (logicalPosition < kGripperCommandMin ||
      logicalPosition > kGripperCommandMax || speed < 1 ||
      speed > 3073 || acceleration < 0 || acceleration > 150) {
    reply("ERROR: position=0..500 speed=1..3073 acc=0..150");
    return;
  }
  if (loadStopLatched) {
    if (logicalPosition >= loadStopCommand - kLoadLatchReleaseCommandDelta) {
      reply("LOAD_STOP latched: move controller toward OPEN");
      return;
    }
    loadStopLatched = false;
    reply("LOAD_STOP released");
  }
  if (servoBus.ping(1) == -1 || servoBus.ping(2) == -1) {
    reply("ERROR: servo ID 1 or 2 was not found");
    return;
  }

  const int servo1Raw =
      kServo1OpenRaw +
      ((kServo1ClosedRaw - kServo1OpenRaw) * logicalPosition) /
          kGripperCommandMax;
  const int servo2Raw =
      kServo2OpenRaw +
      ((kServo2ClosedRaw - kServo2OpenRaw) * logicalPosition) /
          kGripperCommandMax;
  servoBus.syncWriteTwoPositions(
      1,
      static_cast<int16_t>(servo1Raw),
      2,
      static_cast<int16_t>(servo2Raw),
      static_cast<uint16_t>(speed),
      static_cast<uint8_t>(acceleration));
  loadTarget1Raw = servo1Raw;
  loadTarget2Raw = servo2Raw;
  activeGripperCommand = logicalPosition;
  loadHighSampleCount = 0;
  lastLoadPollMs = millis();
  loadProtectionActive = true;
  reply(
      "PAIR_MOVE sent: position=" + String(logicalPosition));
}

void monitorGripperLoad() {
  if (!loadProtectionActive ||
      millis() - lastLoadPollMs < kLoadPollIntervalMs) {
    return;
  }
  lastLoadPollMs = millis();

  const int position1 = servoBus.readPosition(1);
  const int position2 = servoBus.readPosition(2);
  const int current1 = servoBus.readCurrent(1);
  const int current2 = servoBus.readCurrent(2);
  if (position1 < 0 || position2 < 0 || current1 < 0 || current2 < 0) {
    return;
  }

  if (abs(position1 - loadTarget1Raw) <= kTargetPositionTolerance &&
      abs(position2 - loadTarget2Raw) <= kTargetPositionTolerance) {
    loadProtectionActive = false;
    loadHighSampleCount = 0;
    return;
  }

  if (current1 >= kLoadCurrentThreshold || current2 >= kLoadCurrentThreshold) {
    ++loadHighSampleCount;
  } else {
    loadHighSampleCount = 0;
  }
  if (loadHighSampleCount < kLoadConsecutiveSamples) {
    return;
  }

  servoBus.syncWriteTwoPositions(1, position1, 2, position2, 50, 5);
  loadProtectionActive = false;
  loadHighSampleCount = 0;
  loadStopLatched = true;
  loadStopCommand = activeGripperCommand;
  reply(
      "LOAD_STOP hold: id1_raw=" + String(position1) +
      " current=" + String(current1) + ", id2_raw=" + String(position2) +
      " current=" + String(current2));
  showStatus("LOAD STOP", "Holding position");
}

void setServoTorque(const String &arguments, bool enabled) {
  int id = -1;
  if (!parseId(arguments, id)) {
    reply(enabled ? "ERROR: use TORQUE_ON <id>" : "ERROR: use TORQUE_OFF <id>");
    return;
  }

  if (servoBus.ping(static_cast<uint8_t>(id)) == -1) {
    reply("ERROR: servo not found: ID " + String(id));
    return;
  }

  servoBus.enableTorque(static_cast<uint8_t>(id), enabled);
  reply(
      String(enabled ? "TORQUE_ON complete: ID " : "TORQUE_OFF complete: ID ") +
      String(id));
}

void changeServoId(const String &arguments) {
  int currentId = -1;
  int newId = -1;
  const int parsed = sscanf(arguments.c_str(), "%d %d", &currentId, &newId);
  if (parsed != 2 || currentId < 0 || currentId > 253 || newId < 0 ||
      newId > 253 || currentId == newId) {
    reply("ERROR: use SET_ID <current_id> <new_id>, IDs 0..253");
    return;
  }

  if (servoBus.ping(static_cast<uint8_t>(currentId)) == -1) {
    reply("ERROR: current servo not found: ID " + String(currentId));
    return;
  }
  if (servoBus.ping(static_cast<uint8_t>(newId)) != -1) {
    reply("ERROR: new ID is already in use: " + String(newId));
    return;
  }

  servoBus.changeId(
      static_cast<uint8_t>(currentId), static_cast<uint8_t>(newId));
  if (servoBus.ping(static_cast<uint8_t>(newId)) == -1) {
    reply("ERROR: ID change could not be verified");
    return;
  }

  reply("SET_ID complete: " + String(currentId) + " -> " + String(newId));
}

void clearServoOffset(const String &arguments) {
  int id = -1;
  if (!parseId(arguments, id)) {
    reply("ERROR: use CLEAR_OFFSET <id>");
    return;
  }
  if (id == 1 || id == 2) {
    reply("ERROR: offset changes are locked by configured gripper limits");
    return;
  }
  if (servoBus.ping(static_cast<uint8_t>(id)) == -1) {
    reply("ERROR: servo not found: ID " + String(id));
    return;
  }
  servoBus.clearCalibrationOffset(static_cast<uint8_t>(id));
  const int rawPosition = servoBus.readPosition(static_cast<uint8_t>(id));
  reply("CLEAR_OFFSET complete: id=" + String(id) + " raw=" + String(rawPosition));
}

void handleCommand(String command) {
  command.trim();
  if (command.isEmpty()) {
    return;
  }

  Serial.print("Received: ");
  Serial.println(command);

  if (command.equalsIgnoreCase("SET_ZERO")) {
    reply("ERROR: SET_ZERO is locked by configured gripper limits");
    return;
  }

  if (command.equalsIgnoreCase("PING")) {
    reply("PONG");
    return;
  }

  if (command.equalsIgnoreCase("OPEN")) {
    pairMoveServos("0");
    return;
  }

  if (command.equalsIgnoreCase("CLOSE")) {
    pairMoveServos("500");
    return;
  }

  const int separator = command.indexOf(' ');
  const String verb = separator == -1 ? command : command.substring(0, separator);
  const String arguments = separator == -1 ? "" : command.substring(separator + 1);

  if (verb.equalsIgnoreCase("READ")) {
    readServoPosition(arguments);
    return;
  }

  if (verb.equalsIgnoreCase("MOVE")) {
    moveServo(arguments);
    return;
  }

  if (verb.equalsIgnoreCase("RAW_MOVE")) {
    moveServoRaw(arguments);
    return;
  }

  if (verb.equalsIgnoreCase("SYNC_MOVE")) {
    syncMoveServos(arguments);
    return;
  }

  if (verb.equalsIgnoreCase("PAIR_MOVE")) {
    pairMoveServos(arguments);
    return;
  }

  if (verb.equalsIgnoreCase("TORQUE_ON")) {
    setServoTorque(arguments, true);
    return;
  }

  if (verb.equalsIgnoreCase("TORQUE_OFF")) {
    setServoTorque(arguments, false);
    return;
  }

  if (verb.equalsIgnoreCase("SET_ID")) {
    changeServoId(arguments);
    return;
  }

  if (verb.equalsIgnoreCase("CLEAR_OFFSET")) {
    clearServoOffset(arguments);
    return;
  }

  reply("ERROR: use PING, OPEN, CLOSE, SET_ZERO, CLEAR_OFFSET, SET_ID, READ, MOVE, RAW_MOVE, SYNC_MOVE, PAIR_MOVE, TORQUE_ON, or TORQUE_OFF");
}

void readBluetoothCommand() {
  while (SerialBT.available()) {
    const char received = static_cast<char>(SerialBT.read());

    if (received == '\n') {
      handleCommand(bluetoothCommand);
      bluetoothCommand = "";
      continue;
    }

    if (received == '\r') {
      continue;
    }

    if (bluetoothCommand.length() >= kMaxCommandLength) {
      bluetoothCommand = "";
      Serial.println("Bluetooth command rejected: too long");
      SerialBT.println("ERROR: command too long");
      continue;
    }

    bluetoothCommand += received;
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin(kOledSdaPin, kOledSclPin);
  displayAvailable = initializeDisplay();
  showStatus("BOOTING", "STS3215 Gripper");
  delay(500);

  Serial1.begin(
      kServoBaudRate,
      SERIAL_8N1,
      kServoRxPin,
      kServoTxPin);
  bluetoothCommand.reserve(kMaxCommandLength);
  delay(1000);

  showStatus("BT STARTING", kBluetoothName);
  SerialBT.register_callback(bluetoothCallback);
  if (!SerialBT.begin(kBluetoothName)) {
    Serial.println("Bluetooth startup failed");
    showStatus("BT ERROR", "Restart board");
    return;
  }
  bluetoothStarted = true;

  Serial.print("Bluetooth started: ");
  Serial.println(kBluetoothName);
  Serial.println("Commands: PING, OPEN, CLOSE, SET_ZERO, CLEAR_OFFSET, SET_ID, READ, MOVE, RAW_MOVE, SYNC_MOVE, PAIR_MOVE, TORQUE_ON, TORQUE_OFF");
  showStatus("BT READY", "Searching...");
}

void loop() {
  if (bluetoothStateChanged) {
    bluetoothStateChanged = false;
    showStatus(
        bluetoothConnected ? "BT CONNECTED" : "BT READY",
        bluetoothConnected ? "Controller online" : "Searching...");
    if (bluetoothConnected) {
      gripperHomeRequested = true;
    }
  }
  if (gripperHomeRequested) {
    gripperHomeRequested = false;
    pairMoveServos(
        "0 " + String(kConnectMoveSpeed) + " " +
        String(kConnectMoveAcceleration));
  }
  readBluetoothCommand();
  monitorGripperLoad();
}
