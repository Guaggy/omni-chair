#include "safety.h"

#include <Arduino.h>
#include "config.h"
#include "controller.h"

static bool Motors_Enabled = false;
static int Zero_Count = 0;

// Require zero commands at startup and stop on USB timeout
void Update_Motor_Safety(int Front_Left, int Front_Right, int Back_Left, int Back_Right) {
  bool Motors_Zero = Front_Left == 0 && Front_Right == 0 && Back_Left == 0 && Back_Right == 0;

  if (Get_Controller_Mode() == Controller_USB && !USB_Controller_Is_Valid()) {
    if (Motors_Enabled) Serial.println("Joystick timeout - motors stopped");

    Motors_Enabled = false;
    Zero_Count = 0;
    return;
  }

  if (!Motors_Enabled) {
    if (Motors_Zero) Zero_Count++;
    else Zero_Count = 0;

    if (Zero_Count > 10) {
      Motors_Enabled = true;
      Zero_Count = 0;
      Serial.println("Motors enabled");
    }
  }
}

bool Motors_Are_Enabled() {
  return Motors_Enabled;
}

// Print the reason for the most recent ESP32 reset
void Print_Reset_Reason() {
  if (!Debug_Reset_Reason) return;

  esp_reset_reason_t Reason = esp_reset_reason();
  Serial.print("Reset reason: ");

  switch (Reason) {
    case ESP_RST_POWERON:
      Serial.println("POWER ON");
      break;
    case ESP_RST_SW:
      Serial.println("SOFTWARE RESET");
      break;
    case ESP_RST_PANIC:
      Serial.println("PANIC / CRASH");
      break;
    case ESP_RST_INT_WDT:
      Serial.println("INTERRUPT WATCHDOG");
      break;
    case ESP_RST_TASK_WDT:
      Serial.println("TASK WATCHDOG");
      break;
    case ESP_RST_WDT:
      Serial.println("WATCHDOG");
      break;
    case ESP_RST_BROWNOUT:
      Serial.println("BROWNOUT");
      break;
    default:
      Serial.println("Unknown");
      break;
  }
}
