#include <Arduino.h>
#include <BluetoothSerial.h>
#include "SCServo.h"

// Waveshare Servo Driver with ESP32 default ST/STS servo UART pins.
constexpr int kServoRxPin = 18;
constexpr int kServoTxPin = 19;
constexpr uint32_t kServoBaud = 1000000;
constexpr uint8_t kFirstId = 0;
constexpr uint8_t kLastId = 20;
constexpr int kDefaultSpeed = 1000;
constexpr int kDefaultAcc = 50;

BluetoothSerial SerialBT;
SMS_STS sts;
String serialLine;
String btLine;

void reply(const String &message) {
  Serial.println(message);
  SerialBT.println(message);
}

void printHelp() {
  reply("Commands:");
  reply("  PING");
  reply("  SCAN");
  reply("  READ <id>");
  reply("  MOVE <id> <raw> [speed] [acc]");
  reply("  PAIR_MOVE <raw1> <raw2> [speed] [acc]");
  reply("  TORQUE_ON <id>");
  reply("  TORQUE_OFF <id>");
}

void scanServos() {
  reply("Scanning ST/STS servo IDs 0..20 with SCServo library");
  int found = 0;
  for (uint8_t id = kFirstId; id <= kLastId; ++id) {
    if (sts.Ping(id) == -1) {
      delay(10);
      continue;
    }
    ++found;
    const int pos = sts.ReadPos(id);
    reply("FOUND id=" + String(id) + " pos=" + String(pos));
    delay(20);
  }
  if (found == 0) {
    reply("ERROR: no servo found by SCServo library");
  } else {
    reply("SCAN complete: " + String(found) + " servo(s)");
  }
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  if (command.equalsIgnoreCase("PING")) {
    reply("PONG");
    return;
  }
  if (command.equalsIgnoreCase("HELP")) {
    printHelp();
    return;
  }
  if (command.equalsIgnoreCase("SCAN")) {
    scanServos();
    return;
  }

  int id = -1;
  int raw = -1;
  int speed = kDefaultSpeed;
  int acc = kDefaultAcc;

  if (command.startsWith("READ ") || command.startsWith("read ")) {
    if (sscanf(command.c_str(), "%*s %d", &id) != 1) {
      reply("ERROR: use READ <id>");
      return;
    }
    const int pos = sts.ReadPos(id);
    if (pos == -1) {
      reply("ERROR: servo not found: ID " + String(id));
    } else {
      reply("POS id=" + String(id) + " raw=" + String(pos));
    }
    return;
  }

  if (command.startsWith("MOVE ") || command.startsWith("move ")) {
    if (sscanf(command.c_str(), "%*s %d %d %d %d", &id, &raw, &speed, &acc) < 2) {
      reply("ERROR: use MOVE <id> <raw> [speed] [acc]");
      return;
    }
    sts.WritePosEx(id, raw, speed, acc);
    reply("MOVE sent id=" + String(id) + " raw=" + String(raw));
    return;
  }

  if (command.startsWith("PAIR_MOVE ") || command.startsWith("pair_move ")) {
    int raw1 = -1;
    int raw2 = -1;
    if (sscanf(command.c_str(), "%*s %d %d %d %d", &raw1, &raw2, &speed, &acc) < 2) {
      reply("ERROR: use PAIR_MOVE <raw1> <raw2> [speed] [acc]");
      return;
    }
    sts.WritePosEx(1, raw1, speed, acc);
    delay(10);
    sts.WritePosEx(2, raw2, speed, acc);
    reply("PAIR_MOVE sent raw1=" + String(raw1) + " raw2=" + String(raw2));
    return;
  }

  if (command.startsWith("TORQUE_ON ") || command.startsWith("torque_on ")) {
    if (sscanf(command.c_str(), "%*s %d", &id) != 1) {
      reply("ERROR: use TORQUE_ON <id>");
      return;
    }
    sts.EnableTorque(id, 1);
    reply("TORQUE_ON sent id=" + String(id));
    return;
  }

  if (command.startsWith("TORQUE_OFF ") || command.startsWith("torque_off ")) {
    if (sscanf(command.c_str(), "%*s %d", &id) != 1) {
      reply("ERROR: use TORQUE_OFF <id>");
      return;
    }
    sts.EnableTorque(id, 0);
    reply("TORQUE_OFF sent id=" + String(id));
    return;
  }

  reply("ERROR: unknown command. Use HELP");
}

void pollStream(Stream &stream, String &buffer) {
  while (stream.available()) {
    const char c = static_cast<char>(stream.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      handleCommand(buffer);
      buffer = "";
      continue;
    }
    if (buffer.length() < 96) {
      buffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(kServoBaud, SERIAL_8N1, kServoRxPin, kServoTxPin);
  sts.pSerial = &Serial1;
  SerialBT.begin("Waveshare_STS_Diag");

  delay(500);
  reply("Waveshare STS3215 diag ready");
  reply("Servo UART rx=18 tx=19 baud=1000000");
  printHelp();
}

void loop() {
  pollStream(Serial, serialLine);
  pollStream(SerialBT, btLine);
  delay(1);
}
