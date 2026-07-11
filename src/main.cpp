#include <Arduino.h>

#include "app/RobotApplication.h"

robot::RobotApplication application;

void setup() { application.begin(); }
void loop() { application.update(); }
