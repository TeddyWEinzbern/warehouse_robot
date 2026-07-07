#include <Arduino.h>
#include <SoftwareSerial.h>
#include <stdlib.h>

constexpr unsigned long USB_BAUD = 115200;
constexpr unsigned long MOTOR_BAUD = 115200;

constexpr uint8_t MOTOR_RX_PIN = 10;  // Arduino RX <- motor driver TX
constexpr uint8_t MOTOR_TX_PIN = 11;  // Arduino TX -> motor driver RX

constexpr uint8_t MOTOR_MODE = 0;     // 0: 520 motor, 1: TT motor, 2: 310 motor
constexpr uint8_t ENCODER_MODE = 0;   // 0: normal, 1: reversed
constexpr float MAX_MOTOR_SPEED = 0.35f;
constexpr unsigned long COMMAND_TIMEOUT_MS = 300;
constexpr size_t COMMAND_BUFFER_SIZE = 40;

SoftwareSerial motorSerial(MOTOR_RX_PIN, MOTOR_TX_PIN);

char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;
bool commandOverflowed = false;
unsigned long lastCommandAt = 0;
bool motorsStopped = true;

int clampAxis(long value) {
  if (value < -1000) {
    return -1000;
  }
  if (value > 1000) {
    return 1000;
  }
  return static_cast<int>(value);
}

bool parseControlCommand(char *command, int &forward, int &turn, int &strafe) {
  if (command[0] != 'C' || command[1] != ':') {
    return false;
  }

  char *cursor = command + 2;
  char *end = nullptr;

  long parsedForward = strtol(cursor, &end, 10);
  if (end == cursor || *end != ',') {
    return false;
  }

  cursor = end + 1;
  long parsedTurn = strtol(cursor, &end, 10);
  if (end == cursor || *end != ',') {
    return false;
  }

  cursor = end + 1;
  long parsedStrafe = strtol(cursor, &end, 10);
  if (end == cursor || *end != '\0') {
    return false;
  }

  forward = clampAxis(parsedForward);
  turn = clampAxis(parsedTurn);
  strafe = clampAxis(parsedStrafe);
  return true;
}

void sendMotorType(uint8_t motorType) {
  motorSerial.print("$MOTOR_4CH_SET:");
  motorSerial.print(motorType);
  motorSerial.print("!");
}

void sendEncoderPolarity(uint8_t polarity) {
  motorSerial.print("$MOTOR_4CH_SET_ENCPDER_POLARITY:");
  motorSerial.print(polarity);
  motorSerial.print("!");
}

void setMotorSpeed(float speedA, float speedB, float speedC, float speedD) {
  motorSerial.print("$Car:");
  motorSerial.print(speedA, 3);
  motorSerial.print(",");
  motorSerial.print(speedB, 3);
  motorSerial.print(",");
  motorSerial.print(speedC, 3);
  motorSerial.print(",");
  motorSerial.print(speedD, 3);
  motorSerial.print("!");
}

void stopMotors() {
  setMotorSpeed(0.0f, 0.0f, 0.0f, 0.0f);
  motorsStopped = true;
}

void driveMecanum(int forward, int turn, int strafe) {
  float wheelA = forward + strafe + turn;
  float wheelB = forward - strafe - turn;
  float wheelC = forward - strafe + turn;
  float wheelD = forward + strafe - turn;

  float maxMagnitude = max(max(abs(wheelA), abs(wheelB)), max(abs(wheelC), abs(wheelD)));
  if (maxMagnitude > 1000.0f) {
    wheelA = wheelA * 1000.0f / maxMagnitude;
    wheelB = wheelB * 1000.0f / maxMagnitude;
    wheelC = wheelC * 1000.0f / maxMagnitude;
    wheelD = wheelD * 1000.0f / maxMagnitude;
  }

  setMotorSpeed(
      wheelA / 1000.0f * MAX_MOTOR_SPEED,
      wheelB / 1000.0f * MAX_MOTOR_SPEED,
      wheelC / 1000.0f * MAX_MOTOR_SPEED,
      wheelD / 1000.0f * MAX_MOTOR_SPEED);
  motorsStopped = forward == 0 && turn == 0 && strafe == 0;
}

void handleCommand(char *command) {
  int forward = 0;
  int turn = 0;
  int strafe = 0;

  if (!parseControlCommand(command, forward, turn, strafe)) {
    Serial.print("Ignored command: ");
    Serial.println(command);
    return;
  }

  driveMecanum(forward, turn, strafe);
  lastCommandAt = millis();
}

void processUsbSerial() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      if (!commandOverflowed && commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        handleCommand(commandBuffer);
      }
      commandLength = 0;
      commandOverflowed = false;
      continue;
    }

    if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = incoming;
    } else {
      commandLength = 0;
      commandOverflowed = true;
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  motorSerial.begin(MOTOR_BAUD);

  delay(100);
  sendMotorType(MOTOR_MODE);
  delay(100);
  sendEncoderPolarity(ENCODER_MODE);
  delay(100);
  stopMotors();

  lastCommandAt = millis();
  Serial.println("Gamepad mecanum motor bridge ready.");
}

void loop() {
  processUsbSerial();

  if (!motorsStopped && millis() - lastCommandAt > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }
}
