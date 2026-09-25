#include "safety.h"

#include <Arduino.h>
#include "configs/config.h"
#include "controllers/controller.h"
#include "network/log.h"
#include "sensors/collision_sensors.h"
#include "sensors/lidar_sensor.h"

static bool Motors_Enabled = false;
static int Zero_Count = 0;
static SpeedLimit Current_Limit = Limit_None;

// Speed factor per direction actually in use, 1 is no limit
static float Front_Limit = 1, Back_Limit = 1, Left_Limit = 1, Right_Limit = 1, Turn_Limit = 1;
static unsigned long Last_Limit_Update = 0;

bool Hardstop_Enabled = Hardstop_Enabled_At_Start;

// Motors stay off until the joystick works and has been centred for a moment
void Update_Motor_Safety(const int Requested[4]) {
  if (!USB_Controller_Is_Valid()) {
    if (Motors_Enabled) Log_Line("Joystick timeout - motors stopped");
    Motors_Enabled = false;
    Zero_Count = 0;
    return;
  }

  if (Motors_Enabled) return;

  bool Motors_Zero = Requested[0] == 0 && Requested[1] == 0 && Requested[2] == 0 && Requested[3] == 0;
  Zero_Count = Motors_Zero ? Zero_Count + 1 : 0;
  if (Zero_Count > Motor_Enable_Zero_Count) {
    Motors_Enabled = true;
    Zero_Count = 0;
    Log_Line("Motors enabled");
  }
}

bool Motors_Are_Enabled() {
  return Motors_Enabled;
}

void Disable_Motors(const char *Reason) {
  if (Motors_Enabled) Log_Line(String("Motors stopped, ") + Reason);
  Motors_Enabled = false;
  Zero_Count = 0;
}

CollisionZone Classify_Distance(int Distance, int Slow_Distance, int Crawl_Distance) {
  if (Distance <= 0) return Zone_Clear;
  if (Distance <= Crawl_Distance) return Hardstop_Enabled ? Zone_Stop : Zone_Crawl;
  if (Distance <= Slow_Distance) return Zone_Slow;
  return Zone_Clear;
}

CollisionZone Worse_Zone(CollisionZone A, CollisionZone B) {
  return A > B ? A : B;
}

float Zone_Speed_Factor(CollisionZone Zone, float Slow_Factor) {
  switch (Zone) {
    case Zone_Slow: return Slow_Factor;
    case Zone_Crawl: return PSD_Crawl_Factor;
    case Zone_Stop: return 0.0f;
    default: return 1.0f;
  }
}

String Zone_Name(CollisionZone Zone) {
  switch (Zone) {
    case Zone_Slow: return "Slow";
    case Zone_Crawl: return "Crawl";
    case Zone_Stop: return "Stop";
    default: return "Clear";
  }
}

static float PSD_Factor(CollisionZone Zone) {
  return Zone_Speed_Factor(Zone, PSD_Slow_Factor);
}

// A stricter limit applies at once, a looser one eases in so a single clear reading can't make the chair lurch
static void Ease_Limit(float &Limit, float Target, float Step) {
  if (Target < Limit) Limit = Target;
  else Limit = min(Limit + Step, Target);
}

void Limit_Input(ControllerInput &Drive) {
  unsigned long Now = millis();
  float Step = float(Now - Last_Limit_Update) / Limit_Release_Time;
  Last_Limit_Update = Now;

  if (!Motors_Are_Enabled() || !Collision_Enabled) {
    Front_Limit = Back_Limit = Left_Limit = Right_Limit = Turn_Limit = 1;
    return;
  }

  PSD_Zones PSD = Get_PSD_Zones();
  Lidar_Zones Lidar = Get_Lidar_Zones();

  // The stricter of LiDAR and the front PSDs wins, unless the front PSDs are ignored
  float Front_Factor = Get_Lidar_Front_Factor();
  CollisionZone Worst = Worse_Zone(Lidar.Front_Left, Lidar.Front_Right);
  if (!Ignore_Front_PSD) {
    CollisionZone PSD_Front = Worse_Zone(PSD.Front_Left, PSD.Front_Right);
    Front_Factor = min(Front_Factor, PSD_Factor(PSD_Front));
    Worst = Worse_Zone(Worst, PSD_Front);
  }

  // Turning swings the corners, so it slows to crawl when anything is that close but never fully stops
  Worst = Worse_Zone(Worst, Worse_Zone(PSD.Back, Worse_Zone(PSD.Side_Left, PSD.Side_Right)));

  Ease_Limit(Front_Limit, Front_Factor, Step);
  Ease_Limit(Back_Limit, PSD_Factor(PSD.Back), Step);
  Ease_Limit(Right_Limit, PSD_Factor(PSD.Side_Right), Step);
  Ease_Limit(Left_Limit, PSD_Factor(PSD.Side_Left), Step);
  Ease_Limit(Turn_Limit, Worst >= Zone_Crawl ? PSD_Crawl_Factor : 1.0f, Step);

  if (Drive.Y > 0) Drive.Y *= Front_Limit;
  if (Drive.Y < 0) Drive.Y *= Back_Limit;
  if (Drive.X > 0) Drive.X *= Right_Limit;
  if (Drive.X < 0) Drive.X *= Left_Limit;
  Drive.Rotation *= Turn_Limit;
}

const char *Reset_Reason_Text() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "POWER ON";
    case ESP_RST_SW: return "SOFTWARE RESET";
    case ESP_RST_PANIC: return "PANIC / CRASH";
    case ESP_RST_INT_WDT: return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK WATCHDOG";
    case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    default: return "Unknown";
  }
}

void Print_Reset_Reason() {
  if (!Debug_Reset_Reason) return;
  Serial.print("Reset reason: ");
  Serial.println(Reset_Reason_Text());
}

// Look at how much the wheel speeds were actually cut, not just which zone is lit
void Set_Speed_Limit(const int Before[4], const int After[4]) {
  long Sum_Before = 0, Sum_After = 0;
  for (int i = 0; i < 4; i++) {
    Sum_Before += abs(Before[i]);
    Sum_After += abs(After[i]);
  }

  Current_Limit = Limit_None;
  if (Sum_Before == 0) return;

  float Ratio = (float)Sum_After / Sum_Before;
  if (Ratio < 0.02f) Current_Limit = Limit_Stop;
  else if (Ratio < (PSD_Crawl_Factor + PSD_Slow_Factor) / 2) Current_Limit = Limit_Crawl;
  else if (Ratio < 0.98f) Current_Limit = Limit_Slow;
}

SpeedLimit Get_Speed_Limit() {
  return Current_Limit;
}
