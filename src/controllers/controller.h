#pragma once

#include "configs/controller_types.h"

// Start SPI and the USB Host Shield
void Setup_Controllers();

// Poll the shield, but only if it was found at boot
void Update_Controllers();

// Read the joystick, apply deadzone and squaring, and handle button presses
ControllerInput Read_Controller();

// Last joystick reading, used by the web dashboard
ControllerInput Get_Last_Controller_Input();

// True if the shield answered at boot
bool USB_Host_Is_Ready();

// Joystick reports received since boot, for the stats
unsigned long Get_Joystick_Reports();

// True if the shield works and the joystick has reported recently
bool USB_Controller_Is_Valid();
