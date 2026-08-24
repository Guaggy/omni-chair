#include "bluetooth_controller.h"

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "config.h"

static BluetoothSerial Bluetooth;
static String Message;
static unsigned long Watchdog = 0;
static int Angle = 0;
static int Amount = 0;
static int Button = 0;
static float Bluetooth_X = 0;
static float Bluetooth_Y = 0;
static bool Active = false;

// Start the optional Bluetooth controller connection
void Setup_Bluetooth() {
  Bluetooth.begin(Bluetooth_Name);
}

// Read controller messages ending with '#'
void Update_Bluetooth() {
  while (Bluetooth.available()) {
    Active = true;
    Watchdog = millis();
    char Value = Bluetooth.read();

    if (Value == '#') {
      if (Message.length() >= 7) {
        Angle = Message.substring(0, 3).toInt();
        Amount = Message.substring(3, 6).toInt();
        Button = Message.substring(6, 7).toInt();
        Bluetooth_X = Amount * cos(Angle * PI / 180.0);
        Bluetooth_Y = Amount * sin(Angle * PI / 180.0);
      }
      Message = "";
    } else {
      Message += Value;
    }
  }

  if (millis() - Watchdog > Bluetooth_Timeout) {
    Active = false;
    Button = 0;
    Bluetooth_X = 0;
    Bluetooth_Y = 0;
    Message = "";
  }
}

bool Bluetooth_Is_Active() {
  return Active;
}

ControllerInput Read_Bluetooth_Controller() {
  ControllerInput Input;
  Input.X = -Bluetooth_Y / 100;
  Input.Y = Bluetooth_X / 100;
  Input.Button = Button;

  if (Button == 2) Input.Rotation = -1;
  else if (Button == 4) Input.Rotation = 1;
  return Input;
}
