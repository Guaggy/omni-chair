#include "safety.h"

#include <Arduino.h>
#include "config.h"
#include "controller.h"

static bool Motors_Enabled = false;
static int Zero_Count = 0;

// Require zero commands at startup and stop on USB timeout
void Update_Motor_Safety(int Front_Left, int Front_Right, int Back_Left, int Back_Right) {
  bool Motors_Zero = Front_Left == 0 && Front_Right == 0 && Back_Left == 0 && Back_Right == 0;

  if (Get_Controller_Mode() == Controller_USB && Motors_Enabled &&
      millis() - Get_Last_USB_Update() > Joystick_Timeout) {
    Motors_Enabled = false;
    Zero_Count = 0;
    Serial.println("Joystick timeout - motors stopped");
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
