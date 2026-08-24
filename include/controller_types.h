#pragma once

enum ControllerMode {
  Controller_USB,
  Controller_Bluetooth,
  Controller_PS3
};

enum DisplayMenus {
  Default_Menu,
  PSD_Info_Menu,
  LiDAR_Info_Menu,
  Variable_Menu
};

struct ControllerInput {
  float X = 0;
  float Y = 0;
  float Rotation = 0;
  float Speed = 0;
  int Button = 0;
  DisplayMenus Menu = Default_Menu;
};
