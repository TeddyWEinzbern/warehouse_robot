#include <Arduino.h>

#include "RobotController.h"

RobotController robot;

void setup() {
    robot.begin();
}

void loop() {
    robot.update();
}
