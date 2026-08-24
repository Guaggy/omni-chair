#include "motor_control.h"

#include <HardwareSerial.h>
#include "config.h"
#include "pins.h"
#include "safety/safety.h"

static HardwareSerial Robot_Serial(1);

// Send one packetized serial command to a Sabertooth
static void Command(byte Address, byte Command_Byte, byte Value) {
  Robot_Serial.write(Address);
  Robot_Serial.write(Command_Byte);
  Robot_Serial.write(Value);
  Robot_Serial.write((Address + Command_Byte + Value) & 0x7F);
}

// Set the direction and power of one motor channel
static void Motor(byte Address, byte Motor_Number, int Power) {
  if (Motor_Number < 1 || Motor_Number > 2) return;
  Power = constrain(Power, -127, 127);

  byte Command_Byte = 0;
  if (Motor_Number == 1) {
    if (Power >= 0) Command_Byte = 0;
    else Command_Byte = 1;
  }
  if (Motor_Number == 2) {
    if (Power >= 0) Command_Byte = 4;
    else Command_Byte = 5;
  }
  Command(Address, Command_Byte, abs(Power));
}

void Setup_Motors() {
  Robot_Serial.begin(9600, SERIAL_8N1, Motor_RX_Pin, Motor_TX_Pin);
  Robot_Serial.write("R1: 2000\r\n");
  Robot_Serial.write("R2: 2000\r\n");
  delay(50);
}

// Calculate the four mecanum wheel commands
void Calculate_Motor_Speeds(const ControllerInput &Input, int &Front_Left, int &Front_Right,
  int &Back_Left, int &Back_Right) {
  Front_Left = constrain((Input.Y + Input.X - Input.Rotation) * Max_Speed * Input.Speed, -Max_Speed, Max_Speed);
  Front_Right = constrain((Input.Y - Input.X + Input.Rotation) * Max_Speed * Input.Speed, -Max_Speed, Max_Speed);
  Back_Left = constrain((Input.Y - Input.X - Input.Rotation) * Max_Speed * Input.Speed, -Max_Speed, Max_Speed);
  Back_Right = constrain((Input.Y + Input.X + Input.Rotation) * Max_Speed * Input.Speed, -Max_Speed, Max_Speed);
}

// Send wheel speeds to both Sabertooth controllers
void Send_Motor_Speeds(int Front_Left, int Front_Right, int Back_Left, int Back_Right) {
  if (!Motors_Are_Enabled()) {
    Front_Left = 0;
    Front_Right = 0;
    Back_Left = 0;
    Back_Right = 0;
  }
  Motor(129, 1, -Back_Left);
  Motor(129, 2, -Back_Right);
  Motor(128, 1, Front_Right);
  Motor(128, 2, Front_Left);
}

void Set_Ramping(byte Address, byte Value) {
  Command(Address, 16, constrain(Value, 0, 80));
}
