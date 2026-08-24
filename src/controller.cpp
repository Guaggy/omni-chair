#include "controller.h"

#include <Arduino.h>
#include <SPI.h>
#include <Ps3Controller.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include "config.h"
#include "pins.h"
#include "controllers/bluetooth_controller.h"
#include "controllers/usb_joystick.h"

static USB USB_Host;
static USBHub USB_Hub(&USB_Host);
static HIDUniversal USB_HID(&USB_Host);
static JoystickEvents Joystick;
static JoystickReportParser Joystick_Parser(&Joystick);
static ControllerMode Controller_Mode = Controller_USB;
static DisplayMenus Current_Menu = Default_Menu;
static int Previous_Button = 0;
static bool USB_Host_Ready = false;

// Map a value between two floating-point ranges
static float Map_Value(float Value, float Input_Min, float Input_Max, float Output_Min, float Output_Max) {
  return (Value - Input_Min) * (Output_Max - Output_Min) / (Input_Max - Input_Min) + Output_Min;
}

// Start each supported controller interface
void Setup_Controllers() {
  if (Enable_Bluetooth) Setup_Bluetooth();
  Ps3.begin(PS3_Address);

  SPI.begin(USB_Clock_Pin, USB_MISO_Pin, USB_MOSI_Pin, USB_SS_Pin);
  USB_Host_Ready = USB_Host.Init() != -1;
  if (!USB_Host_Ready) Serial.println("USB Host Shield failed");
  delay(200);
  if (!USB_HID.SetReportParser(0, &Joystick_Parser)) {
    USB_Host_Ready = false;
    Serial.println("Joystick parser failed");
  }
}

// Service controllers and select the active input
void Update_Controllers() {
  USB_Host.Task();
  if (Enable_Bluetooth) Update_Bluetooth();

  if (Enable_Bluetooth && Bluetooth_Is_Active()) Controller_Mode = Controller_Bluetooth;
  else if (Ps3.isConnected()) Controller_Mode = Controller_PS3;
  else Controller_Mode = Controller_USB;
}

// Change  once when a menu button is pressed
static void Update_Button(int Button) {
  if (Button != 0 && Previous_Button == 0) {

    // Display Menus
    if (Button == 1) Current_Menu = Default_Menu;
    if (Button == 2) Current_Menu = PSD_Info_Menu;
    if (Button == 3) Current_Menu = LiDAR_Info_Menu;
    if (Button == 4) Current_Menu = Variable_Menu;

    // Other functions
    if (Button == 5) Collision_Enabled = !Collision_Enabled;
  }

  Previous_Button = Button;
}

// Convert the active controller values to the common movement range
ControllerInput Read_Controller() {
  ControllerInput Input;
  int X_Value;
  int Y_Value;
  int Hat;
  int Twist;
  int Slider;
  int Button;

  Joystick.Get_Values(X_Value, Y_Value, Hat, Twist, Slider, Button);

  if (Controller_Mode == Controller_USB) {
    if (X_Value < 499 || X_Value > 520) Input.X = Map_Value(X_Value, 0, 1023, -1, 1);
    if (Y_Value < 499 || Y_Value > 520) Input.Y = Map_Value(Y_Value, 0, 1023, -1, 1);
    if (Twist < 115 || Twist > 130) Input.Rotation = -Map_Value(Twist, 0, 255, -1, 1);
  }
  if (Controller_Mode == Controller_Bluetooth) Input = Read_Bluetooth_Controller();
  if (Controller_Mode == Controller_PS3) {
    Input.X = Map_Value(Ps3.data.analog.stick.lx, -128, 127, -1, 1);
    Input.Y = Map_Value(-Ps3.data.analog.stick.ly, -128, 127, -1, 1);
    Input.Rotation = Map_Value(-Ps3.data.analog.stick.rx, -128, 127, -1, 1);
  }

  if (Square_Inputs) {
    Input.X *= fabs(Input.X);
    Input.Y *= fabs(Input.Y);
    Input.Rotation *= fabs(Input.Rotation);
  }
  if (fabs(Input.X) > fabs(Input.Y)) Input.Y = 0;
  else Input.X = 0;

  Input.Speed = constrain(Map_Value(Slider, 0, 255, 0, 1), 0, 1);

  Update_Button(Button);
  Input.Button = Button;
  Input.Menu = Current_Menu;

  return Input;
}

ControllerMode Get_Controller_Mode() {
  return Controller_Mode;
}

unsigned long Get_Last_USB_Update() {
  return Joystick_Parser.Get_Last_Update_Time();
}

bool USB_Controller_Is_Valid() {
  if (!USB_Host_Ready) return false;
  if (!Joystick_Parser.Has_Valid_Data()) return false;
  return millis() - Get_Last_USB_Update() <= Joystick_Timeout;
}
