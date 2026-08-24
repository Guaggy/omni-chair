#pragma once

#include <Arduino.h>
#include "controller_types.h"

void Setup_Display();
void Update_Display(const ControllerInput &Input, int Front_Left, int Front_Right,
  int Back_Left, int Back_Right);
void Update_Collision_Display(bool Collision_Forward, bool Collision_Left,
  bool Collision_Right, bool Collision_Back);
