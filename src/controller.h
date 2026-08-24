#pragma once
#include "controller_types.h"
void Setup_Controllers();
void Update_Controllers();
ControllerInput Read_Controller();
ControllerMode Get_Controller_Mode();
unsigned long Get_Last_USB_Update();
