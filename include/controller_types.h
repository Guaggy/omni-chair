#pragma once
enum ControllerMode {
  Controller_USB,
  Controller_Bluetooth,
  Controller_PS3
};

struct ControllerInput {
  float X = 0;
  float Y = 0;
  float Rotation = 0;
  float Speed = 0;
  int Button = 0;
};
