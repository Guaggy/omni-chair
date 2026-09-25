#pragma once

#include "configs/controller_types.h"

// Closest obstacle on each side of the front cone in cm, 0 means nothing seen
struct LidarData {
  bool Connected = false;
  int Front_Left = 0;
  int Front_Right = 0;
  int Closest = 0;
};

// One radar point, negative angles are to the left of the chair
struct LidarDisplayPoint {
  int Angle = 0;
  int Distance = 0;
};

// Collision zone for each side of the front cone
struct Lidar_Zones {
  CollisionZone Front_Left = Zone_Clear;
  CollisionZone Front_Right = Zone_Clear;
};

// Call this before display, SPI and USB setup or it hangs
void Setup_Lidar();

// Read new LiDAR data and update the distances after each rotation
void Update_Lidar();
LidarData Get_Lidar_Data();
int Get_Lidar_Display_Points(LidarDisplayPoint *Points, int Max_Points);

// Work out the front zones, reversing clears a stop held for something too close to see
void Update_Lidar_Zones(bool Reversing);
Lidar_Zones Get_Lidar_Zones();

// Speed factor for moving forward, based on what the LiDAR sees
float Get_Lidar_Front_Factor();

// Totals since boot, for the stats
unsigned long Get_Lidar_Rotations();
unsigned long Get_Lidar_Bad_Packets();
