#pragma once

struct LidarData {
  bool Connected = false;
  int Front = 0;
  int Left = 0;
  int Right = 0;
  int Back = 0;
  int Closest = 0;
  bool Warning_Front = false;
  bool Warning_Left = false;
  bool Warning_Right = false;
  bool Warning_Back = false;
};

struct LidarDisplayPoint {
  int Angle = 0;
  int Distance = 0;
};

void Setup_Lidar();
void Update_Lidar();
LidarData Get_Lidar_Data();
int Get_Lidar_Display_Points(LidarDisplayPoint *Points, int Max_Points);
void Check_Lidar_Collisions(int &Front_Left, int &Front_Right,
  int &Back_Left, int &Back_Right);
