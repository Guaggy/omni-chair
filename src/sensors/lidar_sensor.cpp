#include "lidar_sensor.h"

#include <Arduino.h>
#include <driver/uart.h>
#include "configs/config.h"
#include "configs/pins.h"
#include "safety/safety.h"
#include "network/log.h"

// Uses the ESP-IDF UART driver since HardwareSerial hangs without a TX pin
const uart_port_t Lidar_UART = UART_NUM_2;

const int Lidar_Packet_Size = 47;
const int Lidar_Points_Per_Packet = 12;
static uint8_t Lidar_Packet[Lidar_Packet_Size];
static int Lidar_Packet_Index = 0;
static unsigned long Last_Lidar_Packet = 0;
static float Previous_Start_Angle = -1;
static LidarData Lidar_State;
static Lidar_Zones Current_Zones;
static unsigned long Rotation_Count = 0;
static unsigned long Bad_Packet_Count = 0;

// A front half that got too close to see keeps the chair stopped until you reverse
static bool Stop_Held_Left = false;
static bool Stop_Held_Right = false;

// Closest points in the current rotation
static int Scan_Front_Left = 0;
static int Scan_Front_Right = 0;

// When the published distances were last set
static unsigned long Held_Time_Left = 0;
static unsigned long Held_Time_Right = 0;

// Closest distance for every few degrees, starting at the left edge of the cone
const int Lidar_Half_Visible = Lidar_Visible_Angle / 2;
const int Lidar_Radar_Slots = Lidar_Visible_Angle / Lidar_Radar_Point_Step + 1;
static int Scan_Radar_Distances[Lidar_Radar_Slots] = {0};
static int Radar_Distances[Lidar_Radar_Slots] = {0};

// LD06 packet CRC
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

static void Save_Closest(int &Slot, int Distance) {
  if (Slot == 0 || Distance < Slot) Slot = Distance;
}

// Turn the raw angle into the chair's direction and keep the point if it's in the front cone
static void Save_Lidar_Point(float Angle, int Distance) {
  int Adjusted_Angle = (int)(Lidar_Mirror ? 360.0f - Angle : Angle);
  Adjusted_Angle = (Adjusted_Angle + Lidar_Angle_Offset) % 360;
  if (Adjusted_Angle < 0) Adjusted_Angle += 360;

  int Relative_Angle = Adjusted_Angle > 180 ? Adjusted_Angle - 360 : Adjusted_Angle;
  if (abs(Relative_Angle) > Lidar_Half_Visible) return;

  Save_Closest(Scan_Radar_Distances[(Relative_Angle + Lidar_Half_Visible) / Lidar_Radar_Point_Step], Distance);
  if (Relative_Angle < 0) Save_Closest(Scan_Front_Left, Distance);
  else Save_Closest(Scan_Front_Right, Distance);
}

// Keep the closest reading from the last Lidar_Hold_Time ms so one bad rotation can't clear an obstacle
static void Hold_Closest(int &Held, unsigned long &Held_Time, int Scan) {
  bool Expired = millis() - Held_Time > Lidar_Hold_Time;
  if (Scan > 0 && (Held == 0 || Scan <= Held || Expired)) {
    Held = Scan;
    Held_Time = millis();
  }
  else if (Scan == 0 && Expired) {
    Held = 0;
  }
}

// Publish the finished rotation and start a new one
static void Finish_Lidar_Scan() {
  Hold_Closest(Lidar_State.Front_Left, Held_Time_Left, Scan_Front_Left);
  Hold_Closest(Lidar_State.Front_Right, Held_Time_Right, Scan_Front_Right);
  Lidar_State.Closest = 0;
  if (Scan_Front_Left > 0) Save_Closest(Lidar_State.Closest, Scan_Front_Left);
  if (Scan_Front_Right > 0) Save_Closest(Lidar_State.Closest, Scan_Front_Right);

  for (int i = 0; i < Lidar_Radar_Slots; i++) {
    Radar_Distances[i] = Scan_Radar_Distances[i];
    Scan_Radar_Distances[i] = 0;
  }
  Scan_Front_Left = 0;
  Scan_Front_Right = 0;
  Rotation_Count++;
}

// Check the packet and store its 12 measurements
static void Process_Lidar_Packet() {
  if (Lidar_Packet[0] != 0x54 || Lidar_Packet[1] != 0x2C) return;
  if (Calculate_CRC(Lidar_Packet, Lidar_Packet_Size - 1) != Lidar_Packet[Lidar_Packet_Size - 1]) {
    Bad_Packet_Count++;
    return;
  }

  float Start_Angle = (Lidar_Packet[4] | ((uint16_t)Lidar_Packet[5] << 8)) / 100.0f;
  float End_Angle = (Lidar_Packet[42] | ((uint16_t)Lidar_Packet[43] << 8)) / 100.0f;

  // When the angle wraps back past 0 a full rotation is done
  if (Previous_Start_Angle >= 0 && Start_Angle < Previous_Start_Angle) Finish_Lidar_Scan();
  Previous_Start_Angle = Start_Angle;

  float Angle_Range = End_Angle - Start_Angle;
  if (Angle_Range < 0) Angle_Range += 360.0f;

  // Closer points are allowed while the LiDAR guards the front alone
  int Min_Distance = Ignore_Front_PSD ? Lidar_Close_Min_Distance : Lidar_Min_Distance;

  for (int i = 0; i < Lidar_Points_Per_Packet; i++) {
    int Offset = 6 + i * 3;
    int Distance = Lidar_Packet[Offset] | ((uint16_t)Lidar_Packet[Offset + 1] << 8);  // mm
    int Confidence = Lidar_Packet[Offset + 2];
    if (Distance < Min_Distance * 10 || Distance > Lidar_Max_Distance * 10) continue;
    if (Confidence < Lidar_Min_Confidence) continue;

    float Angle = Start_Angle + Angle_Range * i / (Lidar_Points_Per_Packet - 1.0f);
    if (Angle >= 360.0f) Angle -= 360.0f;
    Save_Lidar_Point(Angle, Distance / 10);
  }

  Last_Lidar_Packet = millis();
  Lidar_State.Connected = true;
}

void Setup_Lidar() {
  if (!Enable_Lidar) return;

  // Pull-up so the RX pin doesn't float when no LiDAR is connected
  pinMode(Lidar_RX_Pin, INPUT_PULLUP);

  uart_config_t Config = {};
  Config.baud_rate = Lidar_Baud;
  Config.data_bits = UART_DATA_8_BITS;
  Config.parity = UART_PARITY_DISABLE;
  Config.stop_bits = UART_STOP_BITS_1;
  Config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  Config.source_clk = UART_SCLK_XTAL;

  // Holds about 230 ms of data so nothing is lost during a slow loop
  uart_driver_install(Lidar_UART, 4096, 0, 0, NULL, 0);
  uart_param_config(Lidar_UART, &Config);
  uart_set_pin(Lidar_UART, UART_PIN_NO_CHANGE, Lidar_RX_Pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Collect bytes into a packet, starting at the 0x54 0x2C header
static void Read_Lidar_Byte(uint8_t Value) {
  if (Lidar_Packet_Index == 0 && Value != 0x54) return;
  Lidar_Packet[Lidar_Packet_Index++] = Value;

  if (Lidar_Packet_Index == 2 && Lidar_Packet[1] != 0x2C) {
    Lidar_Packet_Index = 0;
  } else if (Lidar_Packet_Index >= Lidar_Packet_Size) {
    Process_Lidar_Packet();
    Lidar_Packet_Index = 0;
  }
}

void Update_Lidar() {
  if (!Enable_Lidar) return;

  // Read everything waiting, the LD06 sends about 18 KB/s and whatever is left behind gets lost
  uint8_t Buffer[256];
  while (true) {
    int Bytes_Read = uart_read_bytes(Lidar_UART, Buffer, sizeof(Buffer), 0);
    if (Bytes_Read <= 0) break;
    for (int i = 0; i < Bytes_Read; i++) Read_Lidar_Byte(Buffer[i]);
  }

  // If the LiDAR goes quiet, clear everything so old points don't hang around
  if (Lidar_State.Connected && millis() - Last_Lidar_Packet > Lidar_Timeout) {
    Lidar_State = LidarData();
    Previous_Start_Angle = -1;
    Scan_Front_Left = 0;
    Scan_Front_Right = 0;
    for (int i = 0; i < Lidar_Radar_Slots; i++) {
      Scan_Radar_Distances[i] = 0;
      Radar_Distances[i] = 0;
    }
  }

  static unsigned long Last_Lidar_Debug = 0;
  if (Debug_Lidar && millis() - Last_Lidar_Debug >= Debug_Log_Interval) {
    Last_Lidar_Debug = millis();
    Log_Line("LIDAR conn=" + String(Lidar_State.Connected) + " FL=" + String(Lidar_State.Front_Left) +
      " FR=" + String(Lidar_State.Front_Right) + " closest=" + String(Lidar_State.Closest));
  }
}

LidarData Get_Lidar_Data() {
  return Lidar_State;
}

int Get_Lidar_Display_Points(LidarDisplayPoint *Points, int Max_Points) {
  int Count = 0;
  for (int i = 0; i < Lidar_Radar_Slots && Count < Max_Points; i++) {
    if (Radar_Distances[i] == 0) continue;
    Points[Count].Angle = i * Lidar_Radar_Point_Step - Lidar_Half_Visible;
    Points[Count].Distance = Radar_Distances[i];
    Count++;
  }
  return Count;
}

Lidar_Zones Get_Lidar_Zones() {
  return Current_Zones;
}

static void Log_Zone_Change(const char *Name, CollisionZone Old_Zone, CollisionZone New_Zone, int Distance) {
  if (Debug_Collision && New_Zone != Old_Zone) {
    Log_Line(String("LiDAR ") + Name + ": " + Zone_Name(New_Zone) + " (" + String(Distance) + "cm)");
  }
}

// Zone for one front half, with the extra close range steps when the LiDAR guards the front alone
static CollisionZone Front_Half_Zone(int Distance, bool &Stop_Held, bool Reversing) {
  if (!Ignore_Front_PSD) {
    Stop_Held = false;
    return Classify_Distance(Distance, Lidar_Slow_Distance, 0);
  }

  // Stops at the stop distance but only lets go past the release margin, so noise at the edge can't toggle it
  if (Distance > 0 && Distance <= Lidar_Front_Stop_Distance) Stop_Held = true;
  else if (Distance > Lidar_Front_Stop_Distance + Lidar_Stop_Release_Margin) Stop_Held = false;
  else if (Distance <= 0 && Reversing) Stop_Held = false;

  if (Stop_Held) return Zone_Stop;
  if (Distance <= 0) return Zone_Clear;
  if (Distance <= Lidar_Front_Crawl_Distance) return Zone_Crawl;
  if (Distance <= Lidar_Slow_Distance) return Zone_Slow;
  return Zone_Clear;
}

void Update_Lidar_Zones(bool Reversing) {
  Lidar_Zones New_Zones;
  if (Lidar_State.Connected) {
    New_Zones.Front_Left = Front_Half_Zone(Lidar_State.Front_Left, Stop_Held_Left, Reversing);
    New_Zones.Front_Right = Front_Half_Zone(Lidar_State.Front_Right, Stop_Held_Right, Reversing);
  } else if (Ignore_Front_PSD) {
    // Nothing guards the front without the LiDAR, so forward is blocked
    New_Zones.Front_Left = Zone_Stop;
    New_Zones.Front_Right = Zone_Stop;
  }

  Log_Zone_Change("Front-Left", Current_Zones.Front_Left, New_Zones.Front_Left, Lidar_State.Front_Left);
  Log_Zone_Change("Front-Right", Current_Zones.Front_Right, New_Zones.Front_Right, Lidar_State.Front_Right);

  Current_Zones = New_Zones;
}

float Get_Lidar_Front_Factor() {
  CollisionZone Front = Worse_Zone(Current_Zones.Front_Left, Current_Zones.Front_Right);
  if (Front != Zone_Slow) return Zone_Speed_Factor(Front, Lidar_Slow_Factor);

  // Slow has two steps when the LiDAR guards the front alone
  bool Close = Ignore_Front_PSD && Lidar_State.Closest > 0 && Lidar_State.Closest <= Lidar_Front_Slow_Distance;
  return Close ? PSD_Slow_Factor : Lidar_Slow_Factor;
}

unsigned long Get_Lidar_Rotations() {
  return Rotation_Count;
}

unsigned long Get_Lidar_Bad_Packets() {
  return Bad_Packet_Count;
}
