#include "lidar_sensor.h"

#include <Arduino.h>
#include "config.h"
#include "pins.h"
#include "safety/safety.h"

static HardwareSerial Lidar_Serial(2);

const int Lidar_Packet_Size = 47;
static uint8_t Lidar_Packet[Lidar_Packet_Size];
static int Lidar_Packet_Index = 0;
static unsigned long Last_Lidar_Packet = 0;
static unsigned long Last_Lidar_Debug = 0;
static float Previous_Start_Angle = -1;
static LidarData Lidar_State;

static int Scan_Front = 0;
static int Scan_Left = 0;
static int Scan_Right = 0;
static int Scan_Back = 0;
static int Scan_Radar_Distances[360 / Lidar_Radar_Point_Step] = {0};
static int Radar_Distances[360 / Lidar_Radar_Point_Step] = {0};

// Calculate the CRC used by each LD06 packet
static uint8_t Calculate_CRC(const uint8_t *Data, int Length) {
  uint8_t CRC = 0;

  for (int i = 0; i < Length; i++) {
    CRC ^= Data[i];
    for (int Bit = 0; Bit < 8; Bit++) {
      if (CRC & 0x80) CRC = (CRC << 1) ^ 0x4D;
      else CRC <<= 1;
    }
  }

  return CRC;
}

static int Angle_Difference(int Angle, int Centre) {
  int Difference = abs(Angle - Centre);
  if (Difference > 180) Difference = 360 - Difference;
  return Difference;
}

static void Save_Closest(int &Zone, int Distance) {
  if (Zone == 0 || Distance < Zone) Zone = Distance;
}

// Store one measurement in the nearest directional zone
static void Save_Lidar_Point(float Angle, int Distance) {
  int Adjusted_Angle = ((int)Angle + Lidar_Angle_Offset) % 360;
  if (Adjusted_Angle < 0) Adjusted_Angle += 360;

  // Keep one nearby point for each radar angle to limit display work
  int Radar_Index = Adjusted_Angle / Lidar_Radar_Point_Step;
  Save_Closest(Scan_Radar_Distances[Radar_Index], Distance);

  if (Angle_Difference(Adjusted_Angle, Lidar_Front_Angle) <= Lidar_Zone_Half_Width) {
    Save_Closest(Scan_Front, Distance);
  } else if (Angle_Difference(Adjusted_Angle, Lidar_Right_Angle) <= Lidar_Zone_Half_Width) {
    Save_Closest(Scan_Right, Distance);
  } else if (Angle_Difference(Adjusted_Angle, Lidar_Back_Angle) <= Lidar_Zone_Half_Width) {
    Save_Closest(Scan_Back, Distance);
  } else if (Angle_Difference(Adjusted_Angle, Lidar_Left_Angle) <= Lidar_Zone_Half_Width) {
    Save_Closest(Scan_Left, Distance);
  }
}

// Publish one complete rotation for the display and safety checks
static void Finish_Lidar_Scan() {
  Lidar_State.Front = Scan_Front;
  Lidar_State.Left = Scan_Left;
  Lidar_State.Right = Scan_Right;
  Lidar_State.Back = Scan_Back;
  Lidar_State.Closest = 0;

  int Distances[4] = {Scan_Front, Scan_Left, Scan_Right, Scan_Back};
  for (int i = 0; i < 4; i++) {
    if (Distances[i] > 0 && (Lidar_State.Closest == 0 || Distances[i] < Lidar_State.Closest)) {
      Lidar_State.Closest = Distances[i];
    }
  }

  Lidar_State.Warning_Front = Scan_Front > 0 && Scan_Front <= Lidar_Warning_Distance;
  Lidar_State.Warning_Left = Scan_Left > 0 && Scan_Left <= Lidar_Warning_Distance;
  Lidar_State.Warning_Right = Scan_Right > 0 && Scan_Right <= Lidar_Warning_Distance;
  Lidar_State.Warning_Back = Scan_Back > 0 && Scan_Back <= Lidar_Warning_Distance;

  for (int i = 0; i < 360 / Lidar_Radar_Point_Step; i++) {
    Radar_Distances[i] = Scan_Radar_Distances[i];
    Scan_Radar_Distances[i] = 0;
  }

  Scan_Front = 0;
  Scan_Left = 0;
  Scan_Right = 0;
  Scan_Back = 0;
}

// Validate and process the 12 measurements in one LD06 packet
static void Process_Lidar_Packet() {
  if (Lidar_Packet[0] != 0x54 || Lidar_Packet[1] != 0x2C) return;
  if (Calculate_CRC(Lidar_Packet, Lidar_Packet_Size - 1) != Lidar_Packet[Lidar_Packet_Size - 1]) return;

  uint16_t Start_Raw = Lidar_Packet[4] | ((uint16_t)Lidar_Packet[5] << 8);
  uint16_t End_Raw = Lidar_Packet[42] | ((uint16_t)Lidar_Packet[43] << 8);
  float Start_Angle = Start_Raw / 100.0f;
  float End_Angle = End_Raw / 100.0f;

  if (Previous_Start_Angle >= 0 && Start_Angle < Previous_Start_Angle) Finish_Lidar_Scan();
  Previous_Start_Angle = Start_Angle;

  float Angle_Range = End_Angle - Start_Angle;
  if (Angle_Range < 0) Angle_Range += 360.0f;

  for (int i = 0; i < 12; i++) {
    int Offset = 6 + i * 3;
    int Distance = Lidar_Packet[Offset] | ((uint16_t)Lidar_Packet[Offset + 1] << 8);
    int Confidence = Lidar_Packet[Offset + 2];
    if (Distance < Lidar_Min_Distance * 10 || Distance > Lidar_Max_Distance * 10) continue;
    if (Confidence < 5) continue;

    float Angle = Start_Angle + Angle_Range * i / 11.0f;
    if (Angle >= 360.0f) Angle -= 360.0f;
    Save_Lidar_Point(Angle, Distance / 10);
  }

  Last_Lidar_Packet = millis();
  Lidar_State.Connected = true;
}

void Setup_Lidar() {
  if (!Enable_Lidar) return;

  // A larger UART buffer prevents LiDAR data loss during display updates
  Lidar_Serial.setRxBufferSize(1024);
  Lidar_Serial.begin(Lidar_Baud, SERIAL_8N1, Lidar_RX_Pin, Lidar_TX_Pin);
}

// Read a limited number of bytes so joystick and motor updates stay responsive
void Update_Lidar() {
  if (!Enable_Lidar) return;

  int Bytes_Read = 0;
  while (Lidar_Serial.available() && Bytes_Read < 256) {
    uint8_t Value = Lidar_Serial.read();
    Bytes_Read++;

    if (Lidar_Packet_Index == 0 && Value != 0x54) continue;
    Lidar_Packet[Lidar_Packet_Index++] = Value;

    if (Lidar_Packet_Index == 2 && Lidar_Packet[1] != 0x2C) {
      Lidar_Packet_Index = 0;
    } else if (Lidar_Packet_Index >= Lidar_Packet_Size) {
      Process_Lidar_Packet();
      Lidar_Packet_Index = 0;
    }
  }

  if (Lidar_State.Connected && millis() - Last_Lidar_Packet > Lidar_Timeout) {
    Lidar_State = LidarData();
    Previous_Start_Angle = -1;
    Scan_Front = 0;
    Scan_Left = 0;
    Scan_Right = 0;
    Scan_Back = 0;
    for (int i = 0; i < 360 / Lidar_Radar_Point_Step; i++) {
      Scan_Radar_Distances[i] = 0;
      Radar_Distances[i] = 0;
    }
  }

  if (Debug_Lidar && millis() - Last_Lidar_Debug >= Lidar_Debug_Update_Time) {
    Last_Lidar_Debug = millis();
    Serial.print("LIDAR | Front: ");
    Serial.print(Lidar_State.Front);
    Serial.print(" | Left: ");
    Serial.print(Lidar_State.Left);
    Serial.print(" | Right: ");
    Serial.print(Lidar_State.Right);
    Serial.print(" | Back: ");
    Serial.println(Lidar_State.Back);
  }
}

LidarData Get_Lidar_Data() {
  return Lidar_State;
}

int Get_Lidar_Display_Points(LidarDisplayPoint *Points, int Max_Points) {
  int Count = 0;

  for (int i = 0; i < 360 / Lidar_Radar_Point_Step && Count < Max_Points; i++) {
    if (Radar_Distances[i] == 0) continue;

    Points[Count].Angle = i * Lidar_Radar_Point_Step + Lidar_Radar_Point_Step / 2;
    Points[Count].Distance = Radar_Distances[i];
    Count++;
  }

  return Count;
}

// Stop wheel commands that would move toward a close LiDAR obstacle
void Check_Lidar_Collisions(int &Front_Left, int &Front_Right,
  int &Back_Left, int &Back_Right) {
  if (!Motors_Are_Enabled() || !Collision_Enabled || !Lidar_State.Connected) return;

  if (Lidar_State.Front > 0 && Lidar_State.Front <= Lidar_Stop_Distance) {
    if (Front_Left > 0) Front_Left = 0;
    if (Front_Right > 0) Front_Right = 0;
    if (Back_Left > 0) Back_Left = 0;
    if (Back_Right > 0) Back_Right = 0;
  }
  if (Lidar_State.Left > 0 && Lidar_State.Left <= Lidar_Stop_Distance) {
    if (Front_Left < 0) Front_Left = 0;
    if (Front_Right > 0) Front_Right = 0;
    if (Back_Left > 0) Back_Left = 0;
    if (Back_Right < 0) Back_Right = 0;
  }
  if (Lidar_State.Right > 0 && Lidar_State.Right <= Lidar_Stop_Distance) {
    if (Front_Left > 0) Front_Left = 0;
    if (Front_Right < 0) Front_Right = 0;
    if (Back_Left < 0) Back_Left = 0;
    if (Back_Right > 0) Back_Right = 0;
  }
  if (Lidar_State.Back > 0 && Lidar_State.Back <= Lidar_Stop_Distance) {
    if (Front_Left < 0) Front_Left = 0;
    if (Front_Right < 0) Front_Right = 0;
    if (Back_Left < 0) Back_Left = 0;
    if (Back_Right < 0) Back_Right = 0;
  }
}
