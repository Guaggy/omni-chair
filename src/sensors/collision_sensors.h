#pragma once

#include "configs/config.h"
#include "configs/controller_types.h"

// Filtered PSD distances in cm, also used for the raw readings
struct PSD_Distances {
  int Front_Left = PSD_Max_Distance;
  int Front_Right = PSD_Max_Distance;
  int Side_Left = PSD_Max_Distance;
  int Side_Right = PSD_Max_Distance;
  int Back = PSD_Max_Distance;
};

// Collision zone seen by each PSD
struct PSD_Zones {
  CollisionZone Front_Left = Zone_Clear;
  CollisionZone Front_Right = Zone_Clear;
  CollisionZone Side_Left = Zone_Clear;
  CollisionZone Side_Right = Zone_Clear;
  CollisionZone Back = Zone_Clear;
};

// Call this before display, SPI and USB setup or the first reading hangs
void Setup_Collision_Sensors();

// Take a sample every PSD_Read_Interval and update the filtered distances
void Read_Sensors();
PSD_Distances Get_PSD_Distances();

// Last raw ADC reading per sensor, 0-1023
PSD_Distances Get_PSD_Raw();

// Work out the zone for each PSD from the latest distances
void Update_PSD_Zones();
PSD_Zones Get_PSD_Zones();
