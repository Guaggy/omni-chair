#include "stats.h"

#include <WiFi.h>
#include "configs/config.h"
#include "controllers/controller.h"
#include "safety/safety.h"
#include "sensors/lidar_sensor.h"

static SystemStats Stats;
static unsigned long Loop_Count = 0;
static unsigned long Window_Start = 0;
static unsigned long Last_Rotations = 0;
static unsigned long Last_Reports = 0;

void Update_Stats() {
  Loop_Count++;
  unsigned long Elapsed = millis() - Window_Start;
  if (Elapsed < 1000) return;

  unsigned long Rotations = Get_Lidar_Rotations();
  unsigned long Reports = Get_Joystick_Reports();

  Stats.Uptime = millis() / 1000;
  Stats.Loop_Rate = Loop_Count * 1000 / Elapsed;
  Stats.Lidar_Rate = (Rotations - Last_Rotations) * 1000.0f / Elapsed;
  Stats.Lidar_Bad_Packets = Get_Lidar_Bad_Packets();
  Stats.Joystick_Rate = (Reports - Last_Reports) * 1000 / Elapsed;
  Stats.Free_Heap = ESP.getFreeHeap();
  Stats.Free_PSRAM = ESP.getFreePsram();
  Stats.WiFi_Clients = Web_UI_Enabled ? WiFi.softAPgetStationNum() : 0;
  Stats.Reset_Reason = Reset_Reason_Text();

  Loop_Count = 0;
  Window_Start = millis();
  Last_Rotations = Rotations;
  Last_Reports = Reports;
}

SystemStats Get_Stats() {
  return Stats;
}
