#include <Arduino.h>
#include "SCServo.h"

constexpr int kServoRxPin = 18;
constexpr int kServoTxPin = 19;
constexpr uint32_t kServoBaud = 1000000;
constexpr uint8_t kFirstId = 0;
constexpr uint8_t kLastId = 20;

SMS_STS st;
String commandLine;

void scanServos() {
  Serial.println("Scanning ST/STS servo IDs 0..20");
  int found = 0;
  for (uint8_t id = kFirstId; id <= kLastId; ++id) {
    const int pingResult = st.Ping(id);
    if (pingResult == -1) {
      delay(20);
      continue;
    }
    ++found;
    const int pos = st.ReadPos(id);
    Serial.print("FOUND id=");
    Serial.print(id);
    Serial.print(" ping=");
    Serial.print(pingResult);
    Serial.print(" pos=");
    Serial.println(pos);
    delay(50);
  }
  if (found == 0) {
    Serial.println("ERROR: no servo found");
  }
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }
  if (command.equalsIgnoreCase("PING")) {
    Serial.println("PONG");
    return;
  }
  if (command.equalsIgnoreCase("SCAN")) {
    scanServos();
    return;
  }
  if (command.startsWith("READ ") || command.startsWith("read ")) {
    int id = -1;
    if (sscanf(command.c_str(), "%*s %d", &id) != 1) {
      Serial.println("ERROR: use READ <id>");
      return;
    }
    const int pos = st.ReadPos(id);
    Serial.print("READ id=");
    Serial.print(id);
    Serial.print(" pos=");
    Serial.println(pos);
    return;
  }
  if (command.startsWith("MOVE ") || command.startsWith("move ")) {
    int id = -1;
    int pos = -1;
    int speed = 1000;
    int acc = 50;
    if (sscanf(command.c_str(), "%*s %d %d %d %d", &id, &pos, &speed, &acc) < 2) {
      Serial.println("ERROR: use MOVE <id> <raw> [speed] [acc]");
      return;
    }
    st.WritePosEx(id, pos, speed, acc);
    Serial.print("MOVE sent id=");
    Serial.print(id);
    Serial.print(" pos=");
    Serial.println(pos);
    return;
  }
  Serial.println("ERROR: use PING, SCAN, READ <id>, MOVE <id> <raw> [speed] [acc]");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial1.begin(kServoBaud, SERIAL_8N1, kServoRxPin, kServoTxPin);
  st.pSerial = &Serial1;
  delay(1000);
  Serial.println("Waveshare STS3215 USB diag ready");
  Serial.println("Servo UART rx=18 tx=19 baud=1000000");
  Serial.println("Commands: PING, SCAN, READ <id>, MOVE <id> <raw> [speed] [acc]");
}

void loop() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      handleCommand(commandLine);
      commandLine = "";
      continue;
    }
    if (commandLine.length() < 96) {
      commandLine += c;
    }
  }
}
