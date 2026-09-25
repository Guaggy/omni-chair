#pragma once

// All settings in one place, "_At_Start" values are used until a saved setting overwrites them

#include "controller_types.h"

// Debug output to serial and the web log
const bool Debug_Reset_Reason = true;                 // print why the board last reset
const bool Debug_PSD_At_Start = true;                 // PSD distances
const bool Debug_Joystick_At_Start = false;           // decoded joystick values
const bool Debug_Joystick_Raw_At_Start = false;       // raw HID bytes from USB module
const bool Debug_Lidar_At_Start = false;              // LiDAR distances
const bool Debug_Collision_At_Start = false;          // zone status changes
const bool Debug_Motors_At_Start = false;             // wheel commands sent to sabertooth controller
extern bool Debug_PSD;
extern bool Debug_Joystick;
extern bool Debug_Joystick_Raw;
extern bool Debug_Lidar;
extern bool Debug_Collision;
extern bool Debug_Motors;

const unsigned long Debug_Log_Interval = 500;         // ms between repeated debug lines
const unsigned long Task_Watchdog_Timeout = 10000;    // ms before a stuck task resets the board

// Joystick
const bool Enable_USB_Host = true;                    // set to false to skip the shield
const bool Square_Inputs_At_Start = true;             // finer control at low speed, button 9
extern bool Square_Inputs;
const int Joystick_XY_Deadzone = 150;                 // raw X and Y around the centre of 512
const int Joystick_Rotation_Deadzone = 50;            // raw twist around the centre of 127
const unsigned long Joystick_Timeout = 500;           // ms without a report before motors stop

// Motors
const unsigned long Motor_Baud = 9600;
const int Max_Speed = 127;                            // highest speed a Sabertooth accepts
const int Motor_Enable_Zero_Count = 10;               // Runs first loops with a centred joystick before the motors turn on
const unsigned long Ramp_Up_Time = 1000;              // ms from standstill to full speed
const unsigned long Ramp_Down_Time = 300;             // ms from full speed to standstill

// Collision avoidance
const bool Collision_Enabled_At_Start = true;         // button 7, never saved so it always boots from here
extern bool Collision_Enabled;
const bool Hardstop_Enabled_At_Start = false;          // full stop instead of crawl, button 8
extern bool Hardstop_Enabled;
const bool Ignore_Front_PSD_At_Start = false;         // front uses only the LiDAR, button 10
extern bool Ignore_Front_PSD;
const unsigned long Limit_Release_Time = 1000;        // ms for a lifted limit to ease back to full speed, new limits act at once

// LiDAR only looks forward and slows the chair for anything within range
const int Lidar_Slow_Distance = 70;                  // cm
const float Lidar_Slow_Factor = 0.7;

// PSDs look in all directions, where both see something the stricter one wins
const int PSD_Slow_Distance = 40;                     // cm
const float PSD_Slow_Factor = 0.3;
const int PSD_Crawl_Distance = 30;                    // cm, full stop here if hardstop is on
const float PSD_Crawl_Factor = 0.1;                   // also used for turning when anything is at crawl or closer

// LiDAR alone at the front when the front PSDs are ignored, it stops before things get too close for the lidar to see
const int Lidar_Front_Slow_Distance = 70;             // cm, slows to PSD_Slow_Factor
const int Lidar_Front_Crawl_Distance = 55;            // cm, slows to PSD_Crawl_Factor, keep it above the stop distance
const int Lidar_Front_Stop_Distance = 40;             // cm, full stop until you reverse or it moves away
const int Lidar_Stop_Release_Margin = 10;             // cm, the stop only lets go once things are this much farther away
const int Lidar_Close_Min_Distance = 25;              // cm, LiDAR min distance while it guards the front alone, keep it well below the stop distance

// PSD sensors
const int PSD_Min_Distance = 10;                      // cm, the closest the sensor can measure
const int PSD_Max_Distance = 80;                      // cm, the farthest the sensor can measure
const unsigned long PSD_Read_Interval = 20;           // ms between samples
const int PSD_Sample_Count = 3;                       // samples per filtered value
const int PSD_Spike_Trim = 1;                         // samples dropped from each end as spikes
const float PSD_Smoothing = 0.7;                      // how fast the reading follows changes

// LiDAR
const bool Enable_Lidar = true;
const unsigned long Lidar_Baud = 230400;
const unsigned long Lidar_Timeout = 500;              // ms without data before it counts as disconnected
const unsigned long Lidar_Hold_Time = 300;            // ms the closest reading is kept when a rotation misses it
const int Lidar_Min_Distance = 30;                    // cm, closer points are just noise
const int Lidar_Max_Distance = 200;                   // cm, farther points are ignored
const int Lidar_Min_Confidence = 5;                   // weaker points are ignored
const int Lidar_Visible_Angle = 120;                  // degrees the bracket doesn't block
const bool Lidar_Mirror = false;                      // flip if left and right come out swapped
const int Lidar_Angle_Offset = 90;                    // degrees, turns the LiDAR so 0 is the chair's front
const int Lidar_Radar_Point_Step = 3;                 // degrees between radar points on the screens

// Display and web UI
const unsigned long Display_Update_Time = 200;        // ms between screen redraws
const unsigned long Self_Test_Time = 5000;            // ms the boot self test stays on screen, 0 skips it
const char *const AP_Name = "OmniChair";              // open WiFi network name
const char *const Web_Hostname = "omnichair";         // http://omnichair.local
const bool Web_UI_Enabled_At_Start = true;            // button 11
extern bool Web_UI_Enabled;

// Recording for the CSV download, kept in PSRAM and gone after a reboot
const unsigned long Record_Interval = 100;            // ms between rows
const unsigned long Record_Max_Minutes = 15;         // recording stops by itself after this
