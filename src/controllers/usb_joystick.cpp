#include "usb_joystick.h"

#include <Arduino.h>

JoystickReportParser::JoystickReportParser(JoystickEvents *Joystick_Events) {
  Events = Joystick_Events;
}

// Read and store a new USB joystick report
void JoystickReportParser::Parse(USBHID *HID, bool Report_ID, uint8_t Length, uint8_t *Buffer) {
  Last_Update = millis();
  if (Length < Gamepad_Length) return;

  bool Changed = First_Report;
  for (int i = 0; !First_Report && i < Gamepad_Length; i++) {
    if (Buffer[i] != Old_Pad[i]) Changed = true;
  }
  if (!Changed || !Events) return;

  Events->On_Gamepad_Changed((const GamePadEventData *)Buffer);
  for (int i = 0; i < Gamepad_Length; i++) {
    Old_Pad[i] = Buffer[i];
  }
  First_Report = false;
}

unsigned long JoystickReportParser::Get_Last_Update_Time() {
  return Last_Update;
}

// Convert the packed report into normal joystick values
void JoystickEvents::On_Gamepad_Changed(const GamePadEventData *Event) {
  X = Event->x;
  Y = 1023 - Event->y;
  Hat = Event->hat;
  Twist = Event->twist;
  Slider = 255 - Event->slider;
  Button = 0;

  for (int i = 0; i < 8; i++) {
    if (Event->buttonsA & (1 << i)) {
      Button = i + 1;
      break;
    }
  }
  for (int i = 0; Button == 0 && i < 8; i++) {
    if (Event->buttonsB & (1 << i)) {
      Button = i + 9;
      break;
    }
  }
}

void JoystickEvents::Get_Values(int &X_Value, int &Y_Value, int &Hat_Value, int &Twist_Value,
  int &Slider_Value, int &Button_Value) {
  X_Value = X;
  Y_Value = Y;
  Hat_Value = Hat;
  Twist_Value = Twist;
  Slider_Value = Slider;
  Button_Value = Button;
}
