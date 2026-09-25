#include <Arduino.h>
#include <esp_task_wdt.h>
#include "configs/config.h"
#include "controllers/controller.h"
#include "display/display.h"
#include "motors/motor_control.h"
#include "configs/pins.h"
#include "safety/safety.h"
#include "sensors/collision_sensors.h"
#include "sensors/lidar_sensor.h"
#include "settings/settings.h"
#include "network/recorder.h"
#include "network/stats.h"
#include "network/web_server.h"

// Print a boot step and flush it so it still shows up if the board crashes
static void Checkpoint(const char *Message) {
  Serial.println(Message);
  Serial.flush();
}

// Keep the self test on screen for a moment while the joystick and sensors get going
static void Run_Self_Test() {
  unsigned long Start = millis();
  while (millis() - Start < Self_Test_Time) {
    Update_Controllers();
    Read_Sensors();
    Update_Lidar();
    Draw_Self_Test();
    delay(5);
  }
}

void setup() {
  // Give the task watchdog more time so slow startup doesn't reset the board
  esp_task_wdt_config_t Watchdog_Config = {};
  Watchdog_Config.timeout_ms = Task_Watchdog_Timeout;
  Watchdog_Config.idle_core_mask = 0;
  Watchdog_Config.trigger_panic = true;
  esp_task_wdt_reconfigure(&Watchdog_Config);

  pinMode(Peripheral_Power_Enable_Pin, OUTPUT);
  digitalWrite(Peripheral_Power_Enable_Pin, HIGH);

  Serial.begin(115200);
  Serial.println("Booting");
  delay(500);
  Print_Reset_Reason();
  delay(500);

  // Keep this order, LiDAR and PSD setup hang if they run after SPI, display or USB
  Setup_Lidar();
  Checkpoint("CHECKPOINT: Lidar OK");
  Setup_Collision_Sensors();
  Read_Sensors();
  Checkpoint("CHECKPOINT: Collision sensors OK");
  Load_Settings();
  Checkpoint("CHECKPOINT: Settings OK");
  Setup_Motors();
  Checkpoint("CHECKPOINT: Motors OK");
  Setup_Display();
  Checkpoint("CHECKPOINT: Display OK");
  Setup_Controllers();
  Checkpoint("CHECKPOINT: Controllers OK");
  Setup_Web_Server();
  Checkpoint("CHECKPOINT: Web server OK");
  Setup_Recorder();
  Checkpoint("CHECKPOINT: Recorder OK");
  Run_Self_Test();
  Checkpoint("CHECKPOINT: Self test OK");
}

void loop() {
  Update_Controllers();
  ControllerInput Input = Read_Controller();
  Read_Sensors();
  Update_Lidar();
  Update_Web_Server();

  int Requested[4];
  Calculate_Motor_Speeds(Input, Requested);
  Update_Motor_Safety(Requested);

  // Smooth the input first, then limit it for obstacles so the limit acts right away
  ControllerInput Drive = Ramp_Input(Input);
  int Unlimited[4], Wheels[4];
  Calculate_Motor_Speeds(Drive, Unlimited);
  Update_PSD_Zones();
  Update_Lidar_Zones(Input.Y < 0);
  Limit_Input(Drive);
  Calculate_Motor_Speeds(Drive, Wheels);

  Set_Speed_Limit(Unlimited, Wheels);
  Send_Motor_Speeds(Wheels);
  Update_Display(Input, Wheels);
  Update_Recorder(Input, Requested, Wheels);
  Update_Stats();

  static bool First_Loop = true;
  if (First_Loop) Checkpoint("CHECKPOINT: First loop OK");
  First_Loop = false;
}
