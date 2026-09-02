#include <Arduino.h>
#include "controller.h"
#include "display/display.h"
#include "motors/motor_control.h"
#include "safety/safety.h"
#include "sensors/collision_sensors.h"
#include "sensors/lidar_sensor.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  delay(500);
  Print_Reset_Reason();
  delay(500);

  Setup_Motors();
  Setup_Display();
  Setup_Controllers();
  Setup_Collision_Sensors();
  Setup_Lidar();
}

void loop() {
  Update_Controllers();

  ControllerInput Input = Read_Controller();
  Read_Sensors();
  Update_Lidar();

  int Front_Left;
  int Front_Right;
  int Back_Left;
  int Back_Right;

  Calculate_Motor_Speeds(Input, Front_Left, Front_Right, Back_Left, Back_Right);
  Check_Collisions(Front_Left, Front_Right, Back_Left, Back_Right);
  Check_Lidar_Collisions(Front_Left, Front_Right, Back_Left, Back_Right);
  Update_Motor_Safety(Front_Left, Front_Right, Back_Left, Back_Right);
  Send_Motor_Speeds(Front_Left, Front_Right, Back_Left, Back_Right);
  Update_Display(Input, Front_Left, Front_Right, Back_Left, Back_Right);
}
