#pragma once

// Debug
const bool Debug_Reset_Reason = false;
const bool Debug_PSD = false;
const bool Debug_Joystick = false;
const bool Debug_Joystick_Raw = false;

// Controller settings
const bool Square_Inputs = true;
const bool Enable_Bluetooth = false;
const bool Collision_Enabled_At_Start = false;
extern bool Collision_Enabled;

// Maximum motor command accepted by the Sabertooth
const int Max_Speed = 127;

// Distance where movement toward an obstacle is blocked
const int Front_Stop_Distance = 20;
const int Side_Stop_Distance = 20;
const int Back_Stop_Distance = 20;

// Controller watchdog times in milliseconds
const unsigned long Joystick_Timeout = 500;
const unsigned long Bluetooth_Timeout = 60;

// Display update time in milliseconds
const unsigned long Display_Update_Time = 200;

// Controllers
const char PS3_Address[] = "00:00:00:00:00:00";
const char Bluetooth_Name[] = "ODW";

// PSD sensors
const int sensor_max = 80;
const int sensor_min = 10;
const int sensor_average_num = 5;
const int sensor_remove_spike_minmax_num = 1;
