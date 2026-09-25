#include "recorder.h"

#include "configs/config.h"
#include "log.h"
#include "safety/safety.h"
#include "sensors/collision_sensors.h"
#include "sensors/lidar_sensor.h"

// One row, packed small so two hours fit easily in PSRAM
struct RecordRow {
  uint32_t Time;           // ms since the recording started
  int8_t Joystick[4];      // X, Y, rotation and speed in percent
  int8_t Requested[4];     // wheel speeds straight from the joystick
  int8_t Wheels[4];        // wheel speeds sent to the motors
  uint8_t PSD[5];          // cm
  uint8_t Lidar[2];        // cm, front left and right
  uint8_t Zones[7];        // PSD FL, FR, SL, SR, B, then LiDAR FL, FR
  uint8_t Limit;           // SpeedLimit
  uint8_t Flags;           // motors on, collision on, front PSDs ignored
};

const int Max_Rows = Record_Max_Minutes * 60000UL / Record_Interval;

static RecordRow *Rows = nullptr;
static int Row_Count = 0;
static bool Recording = false;
static unsigned long Start_Time = 0;
static unsigned long Last_Row = 0;

void Setup_Recorder() {
  Rows = (RecordRow *)ps_malloc(Max_Rows * sizeof(RecordRow));
  if (!Rows) Log_Line("Recording unavailable, no PSRAM");
}

bool Recorder_Available() {
  return Rows != nullptr;
}

void Start_Recording() {
  if (!Rows) return;
  Row_Count = 0;
  Recording = true;
  Start_Time = millis();
  Last_Row = 0;
  Log_Line("Recording started");
}

void Stop_Recording() {
  if (!Recording) return;
  Recording = false;
  Log_Line("Recording stopped, " + String(Row_Count) + " rows");
}

void Toggle_Recording() {
  if (Recording) Stop_Recording();
  else Start_Recording();
}

bool Is_Recording() {
  return Recording;
}

void Update_Recorder(const ControllerInput &Input, const int Requested[4], const int Wheels[4]) {
  if (!Recording || millis() - Last_Row < Record_Interval) return;
  Last_Row = millis();

  if (Row_Count >= Max_Rows) {
    Recording = false;
    Log_Line("Recording full, stopped at " + String(Row_Count) + " rows");
    return;
  }

  PSD_Distances PSD = Get_PSD_Distances();
  PSD_Zones PSD_Zone = Get_PSD_Zones();
  LidarData Lidar = Get_Lidar_Data();
  Lidar_Zones Lidar_Zone = Get_Lidar_Zones();
  bool Motors_On = Motors_Are_Enabled();

  RecordRow &Row = Rows[Row_Count++];
  Row.Time = millis() - Start_Time;
  Row.Joystick[0] = Input.X * 100;
  Row.Joystick[1] = Input.Y * 100;
  Row.Joystick[2] = Input.Rotation * 100;
  Row.Joystick[3] = Input.Speed * 100;
  for (int i = 0; i < 4; i++) {
    Row.Requested[i] = Requested[i];
    Row.Wheels[i] = Motors_On ? Wheels[i] : 0;
  }
  Row.PSD[0] = PSD.Front_Left;
  Row.PSD[1] = PSD.Front_Right;
  Row.PSD[2] = PSD.Side_Left;
  Row.PSD[3] = PSD.Side_Right;
  Row.PSD[4] = PSD.Back;
  Row.Lidar[0] = Lidar.Front_Left;
  Row.Lidar[1] = Lidar.Front_Right;
  Row.Zones[0] = PSD_Zone.Front_Left;
  Row.Zones[1] = PSD_Zone.Front_Right;
  Row.Zones[2] = PSD_Zone.Side_Left;
  Row.Zones[3] = PSD_Zone.Side_Right;
  Row.Zones[4] = PSD_Zone.Back;
  Row.Zones[5] = Lidar_Zone.Front_Left;
  Row.Zones[6] = Lidar_Zone.Front_Right;
  Row.Limit = Get_Speed_Limit();
  Row.Flags = (Motors_On ? 1 : 0) | (Collision_Enabled ? 2 : 0) | (Ignore_Front_PSD ? 4 : 0);
}

int Recorded_Rows() {
  return Row_Count;
}

unsigned long Recorded_Seconds() {
  if (Recording) return (millis() - Start_Time) / 1000;
  return Row_Count > 0 ? Rows[Row_Count - 1].Time / 1000 : 0;
}

static const char *Limit_Name(uint8_t Limit) {
  switch (Limit) {
    case Limit_Slow: return "Slow";
    case Limit_Crawl: return "Crawl";
    case Limit_Stop: return "Stop";
    default: return "None";
  }
}

String CSV_Header() {
  return "time_ms,joy_x,joy_y,joy_rotation,joy_speed,"
    "req_fl,req_fr,req_bl,req_br,out_fl,out_fr,out_bl,out_br,"
    "psd_fl,psd_fr,psd_sl,psd_sr,psd_b,lidar_fl,lidar_fr,"
    "zone_psd_fl,zone_psd_fr,zone_psd_sl,zone_psd_sr,zone_psd_b,zone_lidar_fl,zone_lidar_fr,"
    "speed_limit,motors_on,collision_on,ignore_front_psd\n";
}

String CSV_Rows(int Start, int Count) {
  String Text;
  int End = min(Start + Count, Row_Count);
  for (int r = Start; r < End; r++) {
    const RecordRow &Row = Rows[r];
    Text += String(Row.Time);
    for (int i = 0; i < 4; i++) Text += "," + String(Row.Joystick[i] / 100.0f, 2);
    for (int i = 0; i < 4; i++) Text += "," + String(Row.Requested[i]);
    for (int i = 0; i < 4; i++) Text += "," + String(Row.Wheels[i]);
    for (int i = 0; i < 5; i++) Text += "," + String(Row.PSD[i]);
    for (int i = 0; i < 2; i++) Text += "," + String(Row.Lidar[i]);
    for (int i = 0; i < 7; i++) Text += "," + Zone_Name((CollisionZone)Row.Zones[i]);
    Text += String(",") + Limit_Name(Row.Limit);
    Text += String(",") + (Row.Flags & 1 ? 1 : 0) + "," + (Row.Flags & 2 ? 1 : 0) + "," + (Row.Flags & 4 ? 1 : 0);
    Text += "\n";
  }
  return Text;
}
