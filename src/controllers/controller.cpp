#include "controller.h"

#include <Arduino.h>
#include <SPI.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include "configs/config.h"
#include "configs/pins.h"
#include "controllers/usb_joystick.h"
#include "network/log.h"
#include "network/recorder.h"
#include "settings/settings.h"

static USB USB_Host;
static USBHub USB_Hub(&USB_Host);
static HIDUniversal USB_HID(&USB_Host);
static JoystickEvents Joystick;
static JoystickReportParser Joystick_Parser(&Joystick);
static DisplayMenus Current_Menu = Default_Menu;
static int Previous_Button = 0;
static bool USB_Host_Ready = false;
static ControllerInput Last_Input;

bool Square_Inputs = Square_Inputs_At_Start;

static float Map_Value(float Value, float Input_Min, float Input_Max, float Output_Min, float Output_Max) {
  return (Value - Input_Min) * (Output_Max - Output_Min) / (Input_Max - Input_Min) + Output_Min;
}

void Setup_Controllers() {
  if (!Enable_USB_Host) {
    Serial.println("USB Host disabled in config.h");
    return;
  }

  SPI.begin(USB_Clock_Pin, USB_MISO_Pin, USB_MOSI_Pin, USB_SS_Pin);
  pinMode(USB_Interrupt_Pin, INPUT);

  USB_Host_Ready = USB_Host.Init() == 0;
  if (!USB_Host_Ready) Log_Line("USB Host Shield failed");
  delay(200);

  if (USB_Host_Ready && !USB_HID.SetReportParser(0, &Joystick_Parser)) {
    USB_Host_Ready = false;
    Log_Line("Joystick parser failed");
  }
}

void Update_Controllers() {
  if (USB_Host_Ready) USB_Host.Task();
}

// Log a setting change and save it, collision is left out since it always boots on
static void Toggle_Setting(bool &Setting, const char *Name, bool Save) {
  Setting = !Setting;
  Log_Line(String(Name) + (Setting ? " ON" : " OFF"));
  if (Save) Save_Settings();
}

// Buttons 1-5 pick the screen, 6 is free, 7-11 toggle settings and 12 starts or stops recording
static void Update_Button(int Button) {
  if (Button != 0 && Previous_Button == 0) {
    if (Button == 1) Current_Menu = Default_Menu;
    if (Button == 2) Current_Menu = PSD_Info_Menu;
    if (Button == 3) Current_Menu = LiDAR_Info_Menu;
    if (Button == 4) Current_Menu = Config_Menu;
    if (Button == 5) Current_Menu = Stats_Menu;

    if (Button == 7) Toggle_Setting(Collision_Enabled, "Collision", false);
    if (Button == 8) Toggle_Setting(Hardstop_Enabled, "Hardstop", true);
    if (Button == 9) Toggle_Setting(Square_Inputs, "Square input", true);
    if (Button == 10) Toggle_Setting(Ignore_Front_PSD, "Ignore front PSD", true);
    if (Button == 11) Toggle_Setting(Web_UI_Enabled, "Web UI", true);
    if (Button == 12) Toggle_Recording();
  }

  Previous_Button = Button;
}

ControllerInput Read_Controller() {
  ControllerInput Input;
  int X_Value, Y_Value, Hat, Twist, Slider, Button;
  Joystick.Get_Values(X_Value, Y_Value, Hat, Twist, Slider, Button);

  if (abs(X_Value - 512) > Joystick_XY_Deadzone) Input.X = Map_Value(X_Value, 0, 1023, -1, 1);
  if (abs(Y_Value - 512) > Joystick_XY_Deadzone) Input.Y = Map_Value(Y_Value, 0, 1023, -1, 1);
  if (abs(Twist - 127) > Joystick_Rotation_Deadzone) Input.Rotation = -Map_Value(Twist, 0, 255, -1, 1);
  Input.Speed = constrain(Map_Value(Slider, 0, 255, 0, 1), 0, 1);

  if (Square_Inputs) {
    Input.X *= fabs(Input.X);
    Input.Y *= fabs(Input.Y);
    Input.Rotation *= fabs(Input.Rotation);
    Input.Speed *= fabs(Input.Speed);
  }

  // Snap the stick to straight, sideways or a 45 degree diagonal
  float Angle = atan2f(fabs(Input.Y), fabs(Input.X));
  if (Angle < PI / 8) Input.Y = 0;
  else if (Angle > (3 * PI) / 8) Input.X = 0;
  else {
    float Magnitude = max(fabs(Input.Y), fabs(Input.X));
    Input.X = Input.X > 0 ? Magnitude : -Magnitude;
    Input.Y = Input.Y > 0 ? Magnitude : -Magnitude;
  }

  Update_Button(Button);
  Input.Button = Button;
  Input.Menu = Current_Menu;

  Last_Input = Input;
  return Input;
}

ControllerInput Get_Last_Controller_Input() {
  return Last_Input;
}

bool USB_Host_Is_Ready() {
  return USB_Host_Ready;
}

unsigned long Get_Joystick_Reports() {
  return Joystick_Parser.Get_Report_Count();
}

bool USB_Controller_Is_Valid() {
  if (!USB_Host_Ready || !Joystick_Parser.Has_Valid_Data()) return false;
  return millis() - Joystick_Parser.Get_Last_Update_Time() <= Joystick_Timeout;
}
