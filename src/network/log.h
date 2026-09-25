#pragma once

#include <Arduino.h>

const int Log_Buffer_Size = 60;  // number of lines kept for the web log

// Print to serial and keep the line for the web log
void Log_Line(const String &Message);

// Get the newest log lines, oldest first
int Get_Log_Lines(String *Lines, int Max_Lines);
