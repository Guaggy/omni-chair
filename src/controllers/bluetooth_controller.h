#pragma once

#include "controller_types.h"

void Setup_Bluetooth();
void Update_Bluetooth();
bool Bluetooth_Is_Active();
ControllerInput Read_Bluetooth_Controller();
