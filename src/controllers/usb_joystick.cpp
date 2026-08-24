#include "usb_joystick.h"

#include <Arduino.h>
#include "config.h"

// Print one byte as eight binary digits
static void Print_Binary_Byte(uint8_t Value) {
  for (int Bit = 7; Bit >= 0; Bit--) {
    Serial.print((Value >> Bit) & 1);
  }
}

// Print raw HID bytes to identify joystick fields
static void Print_Raw_HID(uint8_t Length, uint8_t *Buffer) {
  Serial.print("HID:");
  for (uint8_t i = 0; i < Length; i++) {
    Serial.print(" ");
    if (Buffer[i] < 0x10) Serial.print("0");
    Serial.print(Buffer[i], HEX);
  }
  Serial.println();
}

JoystickReportParser::JoystickReportParser(JoystickEvents *Joystick_Events) {
  Events = Joystick_Events;
}

// Read and store a new USB joystick report
void JoystickReportParser::Parse(USBHID *HID, bool Report_ID, uint8_t Length, uint8_t *Buffer) {
  if (Length < Gamepad_Length) return;

  Last_Update = millis();
  Data_Valid = true;

  bool Changed = First_Report;
  for (int i = 0; !First_Report && i < Gamepad_Length; i++) {
    if (Buffer[i] != Old_Pad[i]) Changed = true;
  }
  if (!Changed || !Events) return;

  if (Debug_Joystick_Raw) Print_Raw_HID(Gamepad_Length, Buffer);
  Events->On_Gamepad_Changed((const GamePadEventData *)Buffer);
  for (int i = 0; i < Gamepad_Length; i++) {
    Old_Pad[i] = Buffer[i];
  }
  First_Report = false;
}

unsigned long JoystickReportParser::Get_Last_Update_Time() {
  return Last_Update;
}

bool JoystickReportParser::Has_Valid_Data() {
  return Data_Valid;
}

// Convert the packed report into normal joystick values
void JoystickEvents::On_Gamepad_Changed(const GamePadEventData *Event) {
  X = Event->x;
  Y = 1023 - Event->y;
  Hat = Event->hat;
  Twist = Event->twist;
  Slider = 255 - Event->slider;
  Buttons_A = Event->buttonsA;
  Buttons_B = Event->buttonsB;
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

  if (Debug_Joystick) Print_Joystick_Debug();
}

// Print interpreted joystick values and both raw button bytes
void JoystickEvents::Print_Joystick_Debug() {
  Serial.print("X: ");
  Serial.print(X);
  Serial.print(" | Y: ");
  Serial.print(Y);
  Serial.print(" | Hat: ");
  Serial.print(Hat);
  Serial.print(" | Twist: ");
  Serial.print(Twist);
  Serial.print(" | Slider: ");
  Serial.print(Slider);
  Serial.print(" | Buttons A: ");
  Print_Binary_Byte(Buttons_A);
  Serial.print(" | Buttons B: ");
  Print_Binary_Byte(Buttons_B);
  Serial.print(" | Button: ");
  Serial.println(Button);
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
