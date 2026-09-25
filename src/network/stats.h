#pragma once

#include <Arduino.h>

// Numbers for the Stats screen and web page, the rates are per second
struct SystemStats {
  unsigned long Uptime = 0;            // s
  int Loop_Rate = 0;
  float Lidar_Rate = 0;                // rotations
  unsigned long Lidar_Bad_Packets = 0; // since boot
  int Joystick_Rate = 0;               // reports
  uint32_t Free_Heap = 0;              // bytes
  uint32_t Free_PSRAM = 0;             // bytes
  int WiFi_Clients = 0;
  const char *Reset_Reason = "";
};

// Call this every loop, the rates are worked out once a second
void Update_Stats();
SystemStats Get_Stats();
