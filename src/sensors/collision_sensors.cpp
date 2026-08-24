#include "collision_sensors.h"

#include <Arduino.h>
#include "config.h"
#include "pins.h"
#include "display/display.h"
#include "safety/safety.h"

static bool Collision_Enabled = false;

// Convert a PSD sensor voltage into distance
static int Read_Sensor(int Pin) {
  float Volts = analogRead(Pin) * 3.3 / 4095.0;
  if (Volts < 0.4) return 80;
  if (Volts > 3.0) return 10;
  return 29.988 * pow(Volts, -1.173);
}

void Setup_Collision_Sensors() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

// Stop wheel commands that would move toward an obstacle
void Check_Collisions(int &Front_Left, int &Front_Right, int &Back_Left, int &Back_Right) {
  int Front_Left_Distance = Read_Sensor(Sensor_Front_Left_Pin);
  int Front_Right_Distance = Read_Sensor(Sensor_Front_Right_Pin);
  int Side_Left_Distance = Read_Sensor(Sensor_Side_Left_Pin);
  int Side_Right_Distance = Read_Sensor(Sensor_Side_Right_Pin);
  int Back_Distance = Read_Sensor(Sensor_Back_Pin);

  bool Collision_Forward = false;
  bool Collision_Left = false;
  bool Collision_Right = false;
  bool Collision_Back = false;

  if (Motors_Are_Enabled() && Collision_Enabled) {
    if (Front_Left_Distance < Front_Stop_Distance || Front_Right_Distance < Front_Stop_Distance) {
      Collision_Forward = true;
      if (Front_Left > 0) Front_Left = 0;
      if (Front_Right > 0) Front_Right = 0;
      if (Back_Left > 0) Back_Left = 0;
      if (Back_Right > 0) Back_Right = 0;
    }
    if (Side_Left_Distance < Side_Stop_Distance) {
      Collision_Left = true;
      if (Front_Left < 0) Front_Left = 0;
      if (Front_Right > 0) Front_Right = 0;
      if (Back_Left > 0) Back_Left = 0;
      if (Back_Right < 0) Back_Right = 0;
    }
    if (Side_Right_Distance < Side_Stop_Distance) {
      Collision_Right = true;
      if (Front_Left > 0) Front_Left = 0;
      if (Front_Right < 0) Front_Right = 0;
      if (Back_Left < 0) Back_Left = 0;
      if (Back_Right > 0) Back_Right = 0;
    }
    if (Back_Distance < Back_Stop_Distance) {
      Collision_Back = true;
      if (Front_Left < 0) Front_Left = 0;
      if (Front_Right < 0) Front_Right = 0;
      if (Back_Left < 0) Back_Left = 0;
      if (Back_Right < 0) Back_Right = 0;
    }
  }

  static bool Old_Collision_Forward = false;
  static bool Old_Collision_Left = false;
  static bool Old_Collision_Right = false;
  static bool Old_Collision_Back = false;

  bool Unchanged = Collision_Forward == Old_Collision_Forward &&
    Collision_Left == Old_Collision_Left &&
    Collision_Right == Old_Collision_Right &&
    Collision_Back == Old_Collision_Back;
  if (Unchanged) return;

  Update_Collision_Display(Collision_Forward, Collision_Left, Collision_Right, Collision_Back);
  if (Collision_Forward) Serial.println("Collision: Forward");
  if (Collision_Left) Serial.println("Collision: Left");
  if (Collision_Right) Serial.println("Collision: Right");
  if (Collision_Back) Serial.println("Collision: Back");

  Old_Collision_Forward = Collision_Forward;
  Old_Collision_Left = Collision_Left;
  Old_Collision_Right = Collision_Right;
  Old_Collision_Back = Collision_Back;
}
