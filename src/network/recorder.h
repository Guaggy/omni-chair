#pragma once

#include <Arduino.h>
#include "configs/controller_types.h"

// Set aside the PSRAM buffer, recording is unavailable if it can't be allocated
void Setup_Recorder();
bool Recorder_Available();

// Starting a new recording throws away the old one
void Start_Recording();
void Stop_Recording();
void Toggle_Recording();
bool Is_Recording();

// Add a row every Record_Interval ms while recording
void Update_Recorder(const ControllerInput &Input, const int Requested[4], const int Wheels[4]);

int Recorded_Rows();
unsigned long Recorded_Seconds();

// The CSV is built in chunks so the whole file never has to fit in memory at once
String CSV_Header();
String CSV_Rows(int Start, int Count);
