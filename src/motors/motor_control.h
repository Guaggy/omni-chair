#pragma once

#include <Arduino.h>
#include "configs/controller_types.h"

void Setup_Motors();

// Smooth the joystick input, returns X, Y and Rotation with the speed already applied
ControllerInput Ramp_Input(const ControllerInput &Input);

// Turn joystick input into four mecanum wheel speeds
void Calculate_Motor_Speeds(const ControllerInput &Input, int Wheels[4]);

// Send the wheel speeds, or zero while the motors are disabled
void Send_Motor_Speeds(const int Wheels[4]);
