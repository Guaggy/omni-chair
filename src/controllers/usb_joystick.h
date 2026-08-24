#pragma once

#include <usbhid.h>

struct __attribute__((packed)) GamePadEventData {
  uint32_t x : 10;
  uint32_t y : 10;
  uint32_t hat : 4;
  uint32_t twist : 8;
  uint8_t buttonsA;
  uint8_t slider;
  uint8_t buttonsB;
};

const int Gamepad_Length = sizeof(GamePadEventData);

class JoystickEvents {
  private:
    int X = 0;
    int Y = 0;
    int Hat = 0;
    int Twist = 0;
    int Slider = 0;
    int Button = 0;
    uint8_t Buttons_A = 0;
    uint8_t Buttons_B = 0;

  public:
    void On_Gamepad_Changed(const GamePadEventData *Event);
    void Get_Values(int &X_Value, int &Y_Value, int &Hat_Value, int &Twist_Value,
      int &Slider_Value, int &Button_Value);
    void Print_Joystick_Debug();
};

class JoystickReportParser : public HIDReportParser {
  private:
    JoystickEvents *Events;
    uint8_t Old_Pad[Gamepad_Length] = {0};
    bool First_Report = true;
    bool Data_Valid = false;
    unsigned long Last_Update = 0;

  public:
    JoystickReportParser(JoystickEvents *Joystick_Events);
    void Parse(USBHID *HID, bool Report_ID, uint8_t Length, uint8_t *Buffer) override;
    unsigned long Get_Last_Update_Time();
    bool Has_Valid_Data();
};
