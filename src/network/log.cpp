#include "log.h"

#include "configs/config.h"

// Debug switches you can flip from the web UI
bool Debug_PSD = Debug_PSD_At_Start;
bool Debug_Joystick = Debug_Joystick_At_Start;
bool Debug_Joystick_Raw = Debug_Joystick_Raw_At_Start;
bool Debug_Lidar = Debug_Lidar_At_Start;
bool Debug_Collision = Debug_Collision_At_Start;
bool Debug_Motors = Debug_Motors_At_Start;

// Keeps the newest log lines for the web page
static String Log_Buffer[Log_Buffer_Size];
static int Log_Count = 0;
static int Log_Next = 0;

void Log_Line(const String &Message) {
  Serial.println(Message);
  Log_Buffer[Log_Next] = Message;
  Log_Next = (Log_Next + 1) % Log_Buffer_Size;
  if (Log_Count < Log_Buffer_Size) Log_Count++;
}

int Get_Log_Lines(String *Lines, int Max_Lines) {
  int Count = min(Log_Count, Max_Lines);
  int Start = (Log_Next - Count + Log_Buffer_Size) % Log_Buffer_Size;
  for (int i = 0; i < Count; i++) {
    Lines[i] = Log_Buffer[(Start + i) % Log_Buffer_Size];
  }
  return Count;
}
