#pragma once

#include <Arduino.h>
#include "configs/controller_types.h"

// How much collision avoidance is slowing the chair, shown on the speed gauge
enum SpeedLimit { Limit_None, Limit_Slow, Limit_Crawl, Limit_Stop };

// Enable motors after a centred joystick and stop them if the joystick goes quiet
void Update_Motor_Safety(const int Requested[4]);
bool Motors_Are_Enabled();

// Turn the motors off, they come back after the joystick has been centred again
void Disable_Motors(const char *Reason);

const char *Reset_Reason_Text();
void Print_Reset_Reason();

// Turn a distance in cm into a zone, 0 means no reading and counts as clear
CollisionZone Classify_Distance(int Distance, int Slow_Distance, int Crawl_Distance);
CollisionZone Worse_Zone(CollisionZone A, CollisionZone B);
float Zone_Speed_Factor(CollisionZone Zone, float Slow_Factor);
String Zone_Name(CollisionZone Zone);

// Slow the movement toward obstacles, per direction, before it's turned into wheel speeds
void Limit_Input(ControllerInput &Drive);

void Set_Speed_Limit(const int Before[4], const int After[4]);
SpeedLimit Get_Speed_Limit();
