#pragma once

struct PSD_Distances {
  int Front_Left = 80;
  int Front_Right = 80;
  int Side_Left = 80;
  int Side_Right = 80;
  int Back = 80;
};

void Read_Sensors();
void Setup_Collision_Sensors();
PSD_Distances Get_PSD_Distances();
void Check_Collisions(int &Front_Left, int &Front_Right, int &Back_Left, int &Back_Right);
