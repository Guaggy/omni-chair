#include "motor_control.h"

#include <HardwareSerial.h>
#include "configs/config.h"
#include "configs/pins.h"
#include "safety/safety.h"
#include "network/log.h"

// Sabertooth addresses, set with the DIP switches
const byte Front_Sabertooth = 128;
const byte Back_Sabertooth = 129;

static HardwareSerial Robot_Serial(1);

// Ramped X, Y and Rotation from -1 to 1, speed included
static float Ramped[3] = {0, 0, 0};
static unsigned long Last_Ramp = 0;

// Send one Sabertooth packet with address, command, value and checksum
static void Command(byte Address, byte Command_Byte, byte Value) {
  Robot_Serial.write(Address);
  Robot_Serial.write(Command_Byte);
  Robot_Serial.write(Value);
  Robot_Serial.write((Address + Command_Byte + Value) & 0x7F);
}

// Command 0 and 1 drive motor 1 forward and back, 4 and 5 do the same for motor 2
static void Motor(byte Address, byte Motor_Number, int Power) {
  Power = constrain(Power, -Max_Speed, Max_Speed);
  byte Command_Byte = (Motor_Number == 1 ? 0 : 4) + (Power < 0 ? 1 : 0);
  Command(Address, Command_Byte, abs(Power));
}

void Setup_Motors() {
  // Pull-up so the RX pin doesn't float when no Sabertooth is connected
  pinMode(Motor_RX_Pin, INPUT_PULLUP);

  Robot_Serial.begin(Motor_Baud, SERIAL_8N1, Motor_RX_Pin, Motor_TX_Pin);
  Robot_Serial.write("R1: 2000\r\n");
  Robot_Serial.write("R2: 2000\r\n");
  delay(50);
}

// Move one value toward its target, faster when slowing down than when speeding up
static float Ramp_Toward(float Current, float Target, float Up_Step, float Down_Step) {
  bool Speeding_Up = (Current >= 0 && Target > Current) || (Current <= 0 && Target < Current);
  float Step = Speeding_Up ? Up_Step : Down_Step;
  if (Target > Current) return min(Current + Step, Target);
  return max(Current - Step, Target);
}

ControllerInput Ramp_Input(const ControllerInput &Input) {
  unsigned long Now = millis();
  float Elapsed = Now - Last_Ramp;
  Last_Ramp = Now;

  // Disabled motors start again from standstill
  if (!Motors_Are_Enabled()) {
    for (int i = 0; i < 3; i++) Ramped[i] = 0;
  }

  float Targets[3] = {Input.X * Input.Speed, Input.Y * Input.Speed, Input.Rotation * Input.Speed};
  float Up_Step = Elapsed / Ramp_Up_Time;
  float Down_Step = Elapsed / Ramp_Down_Time;
  for (int i = 0; i < 3; i++) Ramped[i] = Ramp_Toward(Ramped[i], Targets[i], Up_Step, Down_Step);

  ControllerInput Drive = Input;
  Drive.X = Ramped[0];
  Drive.Y = Ramped[1];
  Drive.Rotation = Ramped[2];
  Drive.Speed = 1;
  return Drive;
}

void Calculate_Motor_Speeds(const ControllerInput &Input, int Wheels[4]) {
  Wheels[Wheel_Front_Left] = constrain((Input.Y + Input.X - Input.Rotation) * Max_Speed * Input.Speed, -Max_Speed, Max_Speed);
  Wheels[Wheel_Front_Right] = constrain((Input.Y - Input.X + Input.Rotation) * Max_Speed * Input.Speed, -Max_Speed, Max_Speed);
  Wheels[Wheel_Back_Left] = constrain((Input.Y - Input.X - Input.Rotation) * Max_Speed * Input.Speed, -Max_Speed, Max_Speed);
  Wheels[Wheel_Back_Right] = constrain((Input.Y + Input.X + Input.Rotation) * Max_Speed * Input.Speed, -Max_Speed, Max_Speed);
}

void Send_Motor_Speeds(const int Wheels[4]) {
  int Out[4] = {0, 0, 0, 0};
  if (Motors_Are_Enabled()) {
    for (int i = 0; i < 4; i++) Out[i] = Wheels[i];
  }

  static unsigned long Last_Motor_Debug = 0;
  if (Debug_Motors && millis() - Last_Motor_Debug >= Debug_Log_Interval) {
    Last_Motor_Debug = millis();
    Log_Line("MOTORS FL=" + String(Out[Wheel_Front_Left]) + " FR=" + String(Out[Wheel_Front_Right]) +
      " BL=" + String(Out[Wheel_Back_Left]) + " BR=" + String(Out[Wheel_Back_Right]));
  }

  // The back motors need inverted commands to drive forward
  Motor(Back_Sabertooth, 1, -Out[Wheel_Back_Left]);
  Motor(Back_Sabertooth, 2, -Out[Wheel_Back_Right]);
  Motor(Front_Sabertooth, 1, Out[Wheel_Front_Right]);
  Motor(Front_Sabertooth, 2, Out[Wheel_Front_Left]);
}
