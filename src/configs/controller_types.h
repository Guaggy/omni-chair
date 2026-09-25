#pragma once

// Screens picked with joystick buttons 1-5
enum DisplayMenus {
  Default_Menu,
  PSD_Info_Menu,
  LiDAR_Info_Menu,
  Config_Menu,
  Stats_Menu
};

// How close something is, from clear to stop
enum CollisionZone {
  Zone_Clear,
  Zone_Slow,   // orange, slowed by a slow factor
  Zone_Crawl,  // red, slowed by PSD_Crawl_Factor
  Zone_Stop    // full stop, only with hardstop on
};

// Order of the wheels in every wheel speed array
enum Wheel {
  Wheel_Front_Left,
  Wheel_Front_Right,
  Wheel_Back_Left,
  Wheel_Back_Right
};

// Joystick input from -1 to 1, speed from 0 to 1
struct ControllerInput {
  float X = 0;
  float Y = 0;
  float Rotation = 0;
  float Speed = 0;
  int Button = 0;
  DisplayMenus Menu = Default_Menu;
};
