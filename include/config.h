#pragma once
// Controller settings
const bool Square_Inputs = true;
const bool Enable_Bluetooth = false;

// Maximum motor command accepted by the Sabertooth
const int Max_Speed = 127;

// Distance where movement toward an obstacle is blocked
const int Front_Stop_Distance = 68;
const int Side_Stop_Distance = 50;
const int Back_Stop_Distance = 60;

// Controller watchdog times in milliseconds
const unsigned long Joystick_Timeout = 500;
const unsigned long Bluetooth_Timeout = 60;

const char PS3_Address[] = "00:00:00:00:00:00";
const char Bluetooth_Name[] = "ODW";
