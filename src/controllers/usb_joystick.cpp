#include "usb_joystick.h"

#include <Arduino.h>
#include "configs/config.h"
#include "network/log.h"

// Print one byte as eight binary digits
static String Binary_Byte(uint8_t Value) {
  String Text = "";
  for (int Bit = 7; Bit >= 0; Bit--) Text += (Value >> Bit) & 1;
  return Text;
}

// Log raw HID bytes to identify joystick fields
static void Print_Raw_HID(uint8_t Length, uint8_t *Buffer) {
  static unsigned long Last_Raw_Debug = 0;
  if (millis() - Last_Raw_Debug < Debug_Log_Interval) return;
  Last_Raw_Debug = millis();

  String Text = "HID:";
  for (uint8_t i = 0; i < Length; i++) {
    Text += " ";
    if (Buffer[i] < 0x10) Text += "0";
    Text += String(Buffer[i], HEX);
  }
  Log_Line(Text);
}

JoystickReportParser::JoystickReportParser(JoystickEvents *Joystick_Events) {
  Events = Joystick_Events;
}

// Read and store a new USB joystick report
void JoystickReportParser::Parse(USBHID *HID, bool Report_ID, uint8_t Length, uint8_t *Buffer) {
  if (Length < Gamepad_Length) return;

  Last_Update = millis();
  Data_Valid = true;
  Report_Count++;

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

unsigned long JoystickReportParser::Get_Report_Count() {
  return Report_Count;
}

// Decode a report, Y and slider are flipped and only the lowest pressed button counts
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

// Log the decoded values and both raw button bytes
void JoystickEvents::Print_Joystick_Debug() {
  static unsigned long Last_Joystick_Debug = 0;
  if (millis() - Last_Joystick_Debug < Debug_Log_Interval) return;
  Last_Joystick_Debug = millis();

  Log_Line("JOY X=" + String(X) + " Y=" + String(Y) + " Hat=" + String(Hat) + " Twist=" + String(Twist) +
    " Slider=" + String(Slider) + " A=" + Binary_Byte(Buttons_A) + " B=" + Binary_Byte(Buttons_B) +
    " Button=" + String(Button));
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
