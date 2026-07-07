#include <Arduino.h>
#include <SoftwareSerial.h>
#include <stdlib.h>

enum class CommandInterface : uint8_t {
  Usb,
  Bluetooth,
  Both,
};

enum class DriveMixing : uint8_t {
  Tank,
  Mecanum,
};

constexpr CommandInterface ACTIVE_COMMAND_INTERFACE = CommandInterface::Both;
constexpr DriveMixing ACTIVE_DRIVE_MIXING = DriveMixing::Mecanum;

constexpr unsigned long USB_BAUD = 115200;
constexpr unsigned long BLUETOOTH_BAUD = 9600;  // HC-05/HC-06 default serial baud.

constexpr uint8_t BLUETOOTH_RX_PIN = A5;  // Arduino RX <- HC-05/HC-06 TXD
constexpr uint8_t BLUETOOTH_TX_PIN = A4;  // Arduino TX -> HC-05/HC-06 RXD

constexpr bool REVERSE_FORWARD_COMMAND = false;
constexpr bool REVERSE_TURN_COMMAND = false;
constexpr bool REVERSE_STRAFE_COMMAND = false;

constexpr uint8_t SHIELD_MOTOR_LATCH_PIN = 12;
constexpr uint8_t SHIELD_MOTOR_CLOCK_PIN = 4;
constexpr uint8_t SHIELD_MOTOR_ENABLE_PIN = 7;
constexpr uint8_t SHIELD_MOTOR_DATA_PIN = 8;

constexpr uint8_t SHIELD_M1_PWM_PIN = 11;
constexpr uint8_t SHIELD_M2_PWM_PIN = 3;
constexpr uint8_t SHIELD_M3_PWM_PIN = 6;
constexpr uint8_t SHIELD_M4_PWM_PIN = 5;

constexpr uint8_t SHIELD_M1_A_LATCH_BIT = 2;
constexpr uint8_t SHIELD_M1_B_LATCH_BIT = 3;
constexpr uint8_t SHIELD_M2_A_LATCH_BIT = 1;
constexpr uint8_t SHIELD_M2_B_LATCH_BIT = 4;
constexpr uint8_t SHIELD_M3_A_LATCH_BIT = 5;
constexpr uint8_t SHIELD_M3_B_LATCH_BIT = 7;
constexpr uint8_t SHIELD_M4_A_LATCH_BIT = 0;
constexpr uint8_t SHIELD_M4_B_LATCH_BIT = 6;

constexpr uint8_t FRONT_LEFT_PWM_PIN = SHIELD_M1_PWM_PIN;
constexpr uint8_t FRONT_LEFT_A_LATCH_BIT = SHIELD_M1_A_LATCH_BIT;
constexpr uint8_t FRONT_LEFT_B_LATCH_BIT = SHIELD_M1_B_LATCH_BIT;
constexpr bool FRONT_LEFT_REVERSE_POLARITY = false;

constexpr uint8_t FRONT_RIGHT_PWM_PIN = SHIELD_M2_PWM_PIN;
constexpr uint8_t FRONT_RIGHT_A_LATCH_BIT = SHIELD_M2_A_LATCH_BIT;
constexpr uint8_t FRONT_RIGHT_B_LATCH_BIT = SHIELD_M2_B_LATCH_BIT;
constexpr bool FRONT_RIGHT_REVERSE_POLARITY = false;

constexpr uint8_t REAR_LEFT_PWM_PIN = SHIELD_M3_PWM_PIN;
constexpr uint8_t REAR_LEFT_A_LATCH_BIT = SHIELD_M3_A_LATCH_BIT;
constexpr uint8_t REAR_LEFT_B_LATCH_BIT = SHIELD_M3_B_LATCH_BIT;
constexpr bool REAR_LEFT_REVERSE_POLARITY = false;

constexpr uint8_t REAR_RIGHT_PWM_PIN = SHIELD_M4_PWM_PIN;
constexpr uint8_t REAR_RIGHT_A_LATCH_BIT = SHIELD_M4_A_LATCH_BIT;
constexpr uint8_t REAR_RIGHT_B_LATCH_BIT = SHIELD_M4_B_LATCH_BIT;
constexpr bool REAR_RIGHT_REVERSE_POLARITY = false;

constexpr uint8_t MAX_MOTOR_PWM = 180;
constexpr uint8_t FRONT_LEFT_MIN_PWM = 55;
constexpr uint8_t FRONT_RIGHT_MIN_PWM = 55;
constexpr uint8_t REAR_LEFT_MIN_PWM = 65;
constexpr uint8_t REAR_RIGHT_MIN_PWM = 65;
constexpr float FRONT_LEFT_SPEED_SCALE = 1.00f;
constexpr float FRONT_RIGHT_SPEED_SCALE = 1.00f;
constexpr float REAR_LEFT_SPEED_SCALE = 1.08f;
constexpr float REAR_RIGHT_SPEED_SCALE = 1.08f;
constexpr unsigned long COMMAND_TIMEOUT_MS = 300;
constexpr bool DEBUG_SERIAL = false;
constexpr unsigned long DEBUG_INTERVAL_MS = 1000;
constexpr size_t COMMAND_BUFFER_SIZE = 40;

struct MotorPins {
  uint8_t pwmPin;
  uint8_t aLatchBit;
  uint8_t bLatchBit;
  bool reversePolarity;
  uint8_t minPwm;
  float speedScale;
};

struct CommandReader {
  char buffer[COMMAND_BUFFER_SIZE];
  size_t length;
  bool overflowed;
};

const MotorPins frontLeftMotor = {
    FRONT_LEFT_PWM_PIN,
    FRONT_LEFT_A_LATCH_BIT,
    FRONT_LEFT_B_LATCH_BIT,
    FRONT_LEFT_REVERSE_POLARITY,
    FRONT_LEFT_MIN_PWM,
    FRONT_LEFT_SPEED_SCALE,
};
const MotorPins frontRightMotor = {
    FRONT_RIGHT_PWM_PIN,
    FRONT_RIGHT_A_LATCH_BIT,
    FRONT_RIGHT_B_LATCH_BIT,
    FRONT_RIGHT_REVERSE_POLARITY,
    FRONT_RIGHT_MIN_PWM,
    FRONT_RIGHT_SPEED_SCALE,
};
const MotorPins rearLeftMotor = {
    REAR_LEFT_PWM_PIN,
    REAR_LEFT_A_LATCH_BIT,
    REAR_LEFT_B_LATCH_BIT,
    REAR_LEFT_REVERSE_POLARITY,
    REAR_LEFT_MIN_PWM,
    REAR_LEFT_SPEED_SCALE,
};
const MotorPins rearRightMotor = {
    REAR_RIGHT_PWM_PIN,
    REAR_RIGHT_A_LATCH_BIT,
    REAR_RIGHT_B_LATCH_BIT,
    REAR_RIGHT_REVERSE_POLARITY,
    REAR_RIGHT_MIN_PWM,
    REAR_RIGHT_SPEED_SCALE,
};

SoftwareSerial bluetoothSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN);

CommandReader usbCommands = {};
CommandReader bluetoothCommands = {};
uint8_t motorLatchState = 0;
unsigned long lastCommandAt = 0;
unsigned long lastDebugAt = 0;
unsigned long usbBytesReceived = 0;
unsigned long bluetoothBytesReceived = 0;
unsigned long usbValidCommands = 0;
unsigned long bluetoothValidCommands = 0;
unsigned long usbIgnoredCommands = 0;
unsigned long bluetoothIgnoredCommands = 0;
const char *lastCommandSource = "none";
char lastCommandKind = '-';
int lastForwardCommand = 0;
int lastTurnCommand = 0;
int lastStrafeCommand = 0;
int lastFrontLeftOutput = 0;
int lastFrontRightOutput = 0;
int lastRearLeftOutput = 0;
int lastRearRightOutput = 0;
bool motorsStopped = true;

bool readsUsbCommands() {
  return ACTIVE_COMMAND_INTERFACE == CommandInterface::Usb ||
         ACTIVE_COMMAND_INTERFACE == CommandInterface::Both;
}

bool readsBluetoothCommands() {
  return ACTIVE_COMMAND_INTERFACE == CommandInterface::Bluetooth ||
         ACTIVE_COMMAND_INTERFACE == CommandInterface::Both;
}

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

bool parseWheelCommand(char *command, int &frontLeft, int &frontRight, int &rearLeft, int &rearRight) {
  if (command[0] != 'W' || command[1] != ':') {
    return false;
  }

  char *cursor = command + 2;
  char *end = nullptr;

  long parsedFrontLeft = strtol(cursor, &end, 10);
  if (end == cursor || *end != ',') {
    return false;
  }

  cursor = end + 1;
  long parsedFrontRight = strtol(cursor, &end, 10);
  if (end == cursor || *end != ',') {
    return false;
  }

  cursor = end + 1;
  long parsedRearLeft = strtol(cursor, &end, 10);
  if (end == cursor || *end != ',') {
    return false;
  }

  cursor = end + 1;
  long parsedRearRight = strtol(cursor, &end, 10);
  if (end == cursor || *end != '\0') {
    return false;
  }

  frontLeft = clampAxis(parsedFrontLeft);
  frontRight = clampAxis(parsedFrontRight);
  rearLeft = clampAxis(parsedRearLeft);
  rearRight = clampAxis(parsedRearRight);
  return true;
}

bool parseMotorCommand(char *command, int &motorIndex, int &speed) {
  if (command[0] != 'M' || command[1] != ':') {
    return false;
  }

  char *cursor = command + 2;
  char *end = nullptr;

  long parsedMotorIndex = strtol(cursor, &end, 10);
  if (end == cursor || *end != ',') {
    return false;
  }

  cursor = end + 1;
  long parsedSpeed = strtol(cursor, &end, 10);
  if (end == cursor || *end != '\0') {
    return false;
  }

  if (parsedMotorIndex < 1 || parsedMotorIndex > 4) {
    return false;
  }

  motorIndex = static_cast<int>(parsedMotorIndex);
  speed = clampAxis(parsedSpeed);
  return true;
}

void updateMotorLatch() {
  digitalWrite(SHIELD_MOTOR_LATCH_PIN, LOW);
  shiftOut(SHIELD_MOTOR_DATA_PIN, SHIELD_MOTOR_CLOCK_PIN, MSBFIRST, motorLatchState);
  digitalWrite(SHIELD_MOTOR_LATCH_PIN, HIGH);
}

void setLatchBit(uint8_t bit, bool enabled) {
  if (enabled) {
    motorLatchState |= static_cast<uint8_t>(1U << bit);
  } else {
    motorLatchState &= static_cast<uint8_t>(~(1U << bit));
  }
}

void configureMotorShield() {
  pinMode(SHIELD_MOTOR_LATCH_PIN, OUTPUT);
  pinMode(SHIELD_MOTOR_CLOCK_PIN, OUTPUT);
  pinMode(SHIELD_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(SHIELD_MOTOR_DATA_PIN, OUTPUT);

  pinMode(SHIELD_M1_PWM_PIN, OUTPUT);
  pinMode(SHIELD_M2_PWM_PIN, OUTPUT);
  pinMode(SHIELD_M3_PWM_PIN, OUTPUT);
  pinMode(SHIELD_M4_PWM_PIN, OUTPUT);

  digitalWrite(SHIELD_MOTOR_ENABLE_PIN, LOW);
  motorLatchState = 0;
  updateMotorLatch();

  analogWrite(SHIELD_M1_PWM_PIN, 0);
  analogWrite(SHIELD_M2_PWM_PIN, 0);
  analogWrite(SHIELD_M3_PWM_PIN, 0);
  analogWrite(SHIELD_M4_PWM_PIN, 0);
}

uint8_t speedToPwm(const MotorPins &motor, float speed) {
  float magnitude = abs(speed) * motor.speedScale;
  if (magnitude < 0.5f) {
    return 0;
  }
  if (magnitude > 1000.0f) {
    magnitude = 1000.0f;
  }

  uint8_t minPwm = min(motor.minPwm, MAX_MOTOR_PWM);
  float pwm = minPwm + (magnitude * (MAX_MOTOR_PWM - minPwm) / 1000.0f);
  if (pwm > MAX_MOTOR_PWM) {
    pwm = MAX_MOTOR_PWM;
  }
  return static_cast<uint8_t>(pwm + 0.5f);
}

void setMotorSpeed(const MotorPins &motor, float speed) {
  uint8_t pwm = speedToPwm(motor, speed);
  if (pwm == 0) {
    analogWrite(motor.pwmPin, 0);
    setLatchBit(motor.aLatchBit, false);
    setLatchBit(motor.bLatchBit, false);
    updateMotorLatch();
    return;
  }

  bool forward = speed > 0.0f;
  if (motor.reversePolarity) {
    forward = !forward;
  }

  setLatchBit(motor.aLatchBit, forward);
  setLatchBit(motor.bLatchBit, !forward);
  updateMotorLatch();
  analogWrite(motor.pwmPin, pwm);
}

void setAllMotorSpeeds(float frontLeft, float frontRight, float rearLeft, float rearRight) {
  setMotorSpeed(frontLeftMotor, frontLeft);
  setMotorSpeed(frontRightMotor, frontRight);
  setMotorSpeed(rearLeftMotor, rearLeft);
  setMotorSpeed(rearRightMotor, rearRight);
  lastFrontLeftOutput = static_cast<int>(frontLeft);
  lastFrontRightOutput = static_cast<int>(frontRight);
  lastRearLeftOutput = static_cast<int>(rearLeft);
  lastRearRightOutput = static_cast<int>(rearRight);
  motorsStopped = frontLeft == 0.0f && frontRight == 0.0f && rearLeft == 0.0f && rearRight == 0.0f;
}

void stopMotors() {
  setAllMotorSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
  motorsStopped = true;
}

void normalizeWheelSpeeds(float &frontLeft, float &frontRight, float &rearLeft, float &rearRight) {
  float maxMagnitude = max(max(abs(frontLeft), abs(frontRight)), max(abs(rearLeft), abs(rearRight)));
  if (maxMagnitude > 1000.0f) {
    frontLeft = frontLeft * 1000.0f / maxMagnitude;
    frontRight = frontRight * 1000.0f / maxMagnitude;
    rearLeft = rearLeft * 1000.0f / maxMagnitude;
    rearRight = rearRight * 1000.0f / maxMagnitude;
  }
}

void driveTank(int forward, int turn) {
  float left = forward + turn;
  float right = forward - turn;
  float frontLeft = left;
  float frontRight = right;
  float rearLeft = left;
  float rearRight = right;

  normalizeWheelSpeeds(frontLeft, frontRight, rearLeft, rearRight);
  setAllMotorSpeeds(frontLeft, frontRight, rearLeft, rearRight);
}

void driveMecanum(int forward, int turn, int strafe) {
  float frontLeft = forward + strafe + turn;
  float frontRight = forward - strafe - turn;
  float rearLeft = forward - strafe + turn;
  float rearRight = forward + strafe - turn;

  normalizeWheelSpeeds(frontLeft, frontRight, rearLeft, rearRight);
  setAllMotorSpeeds(frontLeft, frontRight, rearLeft, rearRight);
}

void driveRobot(int forward, int turn, int strafe) {
  if (REVERSE_FORWARD_COMMAND) {
    forward = -forward;
  }
  if (REVERSE_TURN_COMMAND) {
    turn = -turn;
  }
  if (REVERSE_STRAFE_COMMAND) {
    strafe = -strafe;
  }

  if (ACTIVE_DRIVE_MIXING == DriveMixing::Mecanum) {
    driveMecanum(forward, turn, strafe);
    return;
  }

  driveTank(forward, turn);
}

bool handleCommand(char *command, const char *sourceName) {
  int forward = 0;
  int turn = 0;
  int strafe = 0;

  if (parseControlCommand(command, forward, turn, strafe)) {
    driveRobot(forward, turn, strafe);
    lastCommandKind = 'C';
    lastCommandSource = sourceName;
    lastForwardCommand = forward;
    lastTurnCommand = turn;
    lastStrafeCommand = strafe;
    lastCommandAt = millis();
    return true;
  }

  int frontLeft = 0;
  int frontRight = 0;
  int rearLeft = 0;
  int rearRight = 0;
  if (parseWheelCommand(command, frontLeft, frontRight, rearLeft, rearRight)) {
    setAllMotorSpeeds(frontLeft, frontRight, rearLeft, rearRight);
    lastCommandKind = 'W';
    lastCommandSource = sourceName;
    lastForwardCommand = 0;
    lastTurnCommand = 0;
    lastStrafeCommand = 0;
    lastCommandAt = millis();
    return true;
  }

  int motorIndex = 0;
  int motorSpeed = 0;
  if (parseMotorCommand(command, motorIndex, motorSpeed)) {
    setAllMotorSpeeds(
        motorIndex == 1 ? motorSpeed : 0,
        motorIndex == 2 ? motorSpeed : 0,
        motorIndex == 3 ? motorSpeed : 0,
        motorIndex == 4 ? motorSpeed : 0);
    lastCommandKind = 'M';
    lastCommandSource = sourceName;
    lastForwardCommand = motorIndex;
    lastTurnCommand = motorSpeed;
    lastStrafeCommand = 0;
    lastCommandAt = millis();
    return true;
  }

  Serial.print(F("Ignored "));
  Serial.print(sourceName);
  Serial.print(F(" command: "));
  Serial.println(command);
  return false;
}

void resetCommandReader(CommandReader &reader) {
  reader.length = 0;
  reader.overflowed = false;
}

void processCommandStream(Stream &stream,
                          CommandReader &reader,
                          const char *sourceName,
                          unsigned long &byteCount,
                          unsigned long &validCommandCount,
                          unsigned long &ignoredCommandCount) {
  while (stream.available() > 0) {
    char incoming = static_cast<char>(stream.read());
    byteCount++;

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      if (!reader.overflowed && reader.length > 0) {
        reader.buffer[reader.length] = '\0';
        if (handleCommand(reader.buffer, sourceName)) {
          validCommandCount++;
        } else {
          ignoredCommandCount++;
        }
      }
      resetCommandReader(reader);
      continue;
    }

    if (reader.length < COMMAND_BUFFER_SIZE - 1) {
      reader.buffer[reader.length++] = incoming;
    } else {
      resetCommandReader(reader);
      reader.overflowed = true;
      ignoredCommandCount++;
    }
  }
}

void printDebugStatus() {
  if (!DEBUG_SERIAL || millis() - lastDebugAt < DEBUG_INTERVAL_MS) {
    return;
  }

  lastDebugAt = millis();
  Serial.print(F("DBG usb bytes="));
  Serial.print(usbBytesReceived);
  Serial.print(F(" ok="));
  Serial.print(usbValidCommands);
  Serial.print(F(" bad="));
  Serial.print(usbIgnoredCommands);
  Serial.print(F(" | bt bytes="));
  Serial.print(bluetoothBytesReceived);
  Serial.print(F(" ok="));
  Serial.print(bluetoothValidCommands);
  Serial.print(F(" bad="));
  Serial.print(bluetoothIgnoredCommands);
  Serial.print(F(" | last "));
  Serial.print(lastCommandSource);
  Serial.print(F(" "));
  Serial.print(lastCommandKind);
  Serial.print(F(":"));
  Serial.print(lastForwardCommand);
  Serial.print(F(","));
  Serial.print(lastTurnCommand);
  Serial.print(F(","));
  Serial.print(lastStrafeCommand);
  Serial.print(F(" wheels FL/FR/RL/RR="));
  Serial.print(lastFrontLeftOutput);
  Serial.print(F("/"));
  Serial.print(lastFrontRightOutput);
  Serial.print(F("/"));
  Serial.print(lastRearLeftOutput);
  Serial.print(F("/"));
  Serial.print(lastRearRightOutput);
  Serial.print(F(" stopped="));
  Serial.println(motorsStopped ? F("yes") : F("no"));
}

void setup() {
  Serial.begin(USB_BAUD);
  if (readsBluetoothCommands()) {
    bluetoothSerial.begin(BLUETOOTH_BAUD);
  }

  configureMotorShield();
  stopMotors();

  lastCommandAt = millis();
  Serial.println(F("L293D motor bridge ready."));
  Serial.println(F("Motor shield: L293D V1-compatible M1/M2/M3/M4"));
  Serial.println(F("Drive mixing: Mecanum"));
  Serial.println(F("Bluetooth serial: Arduino RX=A5 <- module TX, Arduino TX=A4 -> module RX"));
}

void loop() {
  if (readsUsbCommands()) {
    processCommandStream(Serial, usbCommands, "USB", usbBytesReceived, usbValidCommands, usbIgnoredCommands);
  }
  if (readsBluetoothCommands()) {
    processCommandStream(bluetoothSerial,
                         bluetoothCommands,
                         "Bluetooth",
                         bluetoothBytesReceived,
                         bluetoothValidCommands,
                         bluetoothIgnoredCommands);
  }

  if (!motorsStopped && millis() - lastCommandAt > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }

  printDebugStatus();
}
