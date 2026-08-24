#pragma once

#include <Arduino.h>
#include "controller_types.h"

void Setup_Motors();
void Calculate_Motor_Speeds(const ControllerInput &Input, int &Front_Left, int &Front_Right,
  int &Back_Left, int &Back_Right);
void Send_Motor_Speeds(int Front_Left, int Front_Right, int Back_Left, int Back_Right);
void Set_Ramping(byte Address, byte Value);
