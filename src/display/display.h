#pragma once

#include <Arduino.h>
#include "configs/controller_types.h"

void Setup_Display();

// Show the boot self test, call it repeatedly while the sensors start up
void Draw_Self_Test();
void Update_Display(const ControllerInput &Input, const int Wheels[4]);
