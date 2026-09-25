#include "collision_sensors.h"

#include <Arduino.h>
#include "configs/config.h"
#include "configs/pins.h"
#include "safety/safety.h"
#include "network/log.h"

enum { Sensor_FL, Sensor_FR, Sensor_SL, Sensor_SR, Sensor_Back, Sensor_Count };

const int Sensor_Pins[Sensor_Count] = {
  Sensor_Front_Left_Pin,
  Sensor_Front_Right_Pin,
  Sensor_Side_Left_Pin,
  Sensor_Side_Right_Pin,
  Sensor_Back_Pin
};

bool Collision_Enabled = Collision_Enabled_At_Start;
bool Ignore_Front_PSD = Ignore_Front_PSD_At_Start;

static int Samples[Sensor_Count][PSD_Sample_Count];
static int Last_Raw[Sensor_Count];
static int Sample_Index = 0;
static float Filtered_Distance[Sensor_Count];
static unsigned long Last_Sensor_Read = 0;
static PSD_Zones Current_Zones;

// Convert a raw reading to cm using the Sharp sensor curve
static int Read_Sensor(int Index) {
  Last_Raw[Index] = analogRead(Sensor_Pins[Index]);
  int Divisor = Last_Raw[Index] - 20;
  if (Divisor <= 0) return PSD_Max_Distance;
  return constrain(4800 / Divisor, PSD_Min_Distance, PSD_Max_Distance);
}

// Sort the samples, drop the highest and lowest and average the rest
static float Trimmed_Average(int *Values) {
  for (int i = 0; i < PSD_Sample_Count - 1; i++) {
    for (int j = 0; j < PSD_Sample_Count - i - 1; j++) {
      if (Values[j] > Values[j + 1]) {
        int Temp = Values[j];
        Values[j] = Values[j + 1];
        Values[j + 1] = Temp;
      }
    }
  }

  int Sum = 0;
  for (int i = PSD_Spike_Trim; i < PSD_Sample_Count - PSD_Spike_Trim; i++) Sum += Values[i];
  return float(Sum) / float(PSD_Sample_Count - 2 * PSD_Spike_Trim);
}

void Setup_Collision_Sensors() {
  for (int i = 0; i < Sensor_Count; i++) Filtered_Distance[i] = PSD_Max_Distance;
  analogReadResolution(10);
}

void Read_Sensors() {
  if (millis() - Last_Sensor_Read < PSD_Read_Interval) return;
  Last_Sensor_Read = millis();

  for (int i = 0; i < Sensor_Count; i++) Samples[i][Sample_Index] = Read_Sensor(i);
  if (++Sample_Index < PSD_Sample_Count) return;
  Sample_Index = 0;

  for (int i = 0; i < Sensor_Count; i++) {
    Filtered_Distance[i] = Filtered_Distance[i] * (1 - PSD_Smoothing) + Trimmed_Average(Samples[i]) * PSD_Smoothing;
  }

  static unsigned long Last_PSD_Debug = 0;
  if (Debug_PSD && millis() - Last_PSD_Debug >= Debug_Log_Interval) {
    Last_PSD_Debug = millis();
    Log_Line("PSD FL=" + String(Filtered_Distance[Sensor_FL], 0) + " FR=" + String(Filtered_Distance[Sensor_FR], 0) +
      " SL=" + String(Filtered_Distance[Sensor_SL], 0) + " SR=" + String(Filtered_Distance[Sensor_SR], 0) +
      " B=" + String(Filtered_Distance[Sensor_Back], 0));
  }
}

PSD_Distances Get_PSD_Distances() {
  PSD_Distances Distances;
  Distances.Front_Left = Filtered_Distance[Sensor_FL];
  Distances.Front_Right = Filtered_Distance[Sensor_FR];
  Distances.Side_Left = Filtered_Distance[Sensor_SL];
  Distances.Side_Right = Filtered_Distance[Sensor_SR];
  Distances.Back = Filtered_Distance[Sensor_Back];
  return Distances;
}

PSD_Distances Get_PSD_Raw() {
  PSD_Distances Raw;
  Raw.Front_Left = Last_Raw[Sensor_FL];
  Raw.Front_Right = Last_Raw[Sensor_FR];
  Raw.Side_Left = Last_Raw[Sensor_SL];
  Raw.Side_Right = Last_Raw[Sensor_SR];
  Raw.Back = Last_Raw[Sensor_Back];
  return Raw;
}

PSD_Zones Get_PSD_Zones() {
  return Current_Zones;
}

static void Log_Zone_Change(const char *Name, CollisionZone Old_Zone, CollisionZone New_Zone, int Distance) {
  if (Debug_Collision && New_Zone != Old_Zone) {
    Log_Line(String("PSD ") + Name + ": " + Zone_Name(New_Zone) + " (" + String(Distance) + "cm)");
  }
}

void Update_PSD_Zones() {
  PSD_Distances Distances = Get_PSD_Distances();

  PSD_Zones New_Zones;
  New_Zones.Front_Left = Classify_Distance(Distances.Front_Left, PSD_Slow_Distance, PSD_Crawl_Distance);
  New_Zones.Front_Right = Classify_Distance(Distances.Front_Right, PSD_Slow_Distance, PSD_Crawl_Distance);
  New_Zones.Side_Left = Classify_Distance(Distances.Side_Left, PSD_Slow_Distance, PSD_Crawl_Distance);
  New_Zones.Side_Right = Classify_Distance(Distances.Side_Right, PSD_Slow_Distance, PSD_Crawl_Distance);
  New_Zones.Back = Classify_Distance(Distances.Back, PSD_Slow_Distance, PSD_Crawl_Distance);

  Log_Zone_Change("Front-Left", Current_Zones.Front_Left, New_Zones.Front_Left, Distances.Front_Left);
  Log_Zone_Change("Front-Right", Current_Zones.Front_Right, New_Zones.Front_Right, Distances.Front_Right);
  Log_Zone_Change("Side-Left", Current_Zones.Side_Left, New_Zones.Side_Left, Distances.Side_Left);
  Log_Zone_Change("Side-Right", Current_Zones.Side_Right, New_Zones.Side_Right, Distances.Side_Right);
  Log_Zone_Change("Back", Current_Zones.Back, New_Zones.Back, Distances.Back);

  Current_Zones = New_Zones;
}
