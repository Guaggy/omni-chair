#pragma once

#include "controller_types.h"

// Debug
const bool Debug_Reset_Reason = false;
const bool Debug_PSD = false;
const bool Debug_Joystick = false;
const bool Debug_Joystick_Raw = false;
const bool Debug_Lidar = false;

// Controller settings
const bool Square_Inputs = true;
const bool Enable_Bluetooth = false;
const bool Collision_Enabled_At_Start = false;
extern bool Collision_Enabled;
const ControllerMode Controller_Mode_Default = Controller_USB;

// LiDAR settings
const bool Enable_Lidar = true;
const unsigned long Lidar_Baud = 230400;
const unsigned long Lidar_Timeout = 500;
const unsigned long Lidar_Debug_Update_Time = 500;
const int Lidar_Min_Distance = 5;
const int Lidar_Max_Distance = 200;
const int Lidar_Warning_Distance = 100;
const int Lidar_Stop_Distance = 50;
const int Lidar_Zone_Half_Width = 45;
const int Lidar_Front_Angle = 0;
const int Lidar_Right_Angle = 90;
const int Lidar_Back_Angle = 180;
const int Lidar_Left_Angle = 270;
const int Lidar_Angle_Offset = 0;
const int Lidar_Radar_Radius = 42;
const int Lidar_Radar_Point_Step = 10;

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
