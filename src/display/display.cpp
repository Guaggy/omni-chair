#include "display.h"

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include "configs/config.h"
#include "controllers/controller.h"
#include "safety/safety.h"
#include "sensors/collision_sensors.h"
#include "sensors/lidar_sensor.h"
#include "network/recorder.h"
#include "network/stats.h"
#include "settings/settings.h"

// Screen is 320x170 in landscape, layout numbers below are in pixels
const int Screen_Width = 320;
const int Screen_Height = 170;

static TFT_eSPI TFT(170, 320);
static TFT_eSprite Display_Sprite(&TFT);
static unsigned long Display_Update_Timer = 0;

// Driving screen with the chair on the left and the speed gauge on the right
const int Wheel_Width = 26;
const int Wheel_Height = 48;
const int Wheel_Left_X = 70;
const int Wheel_Right_X = 178;
const int Wheel_Front_Y = 25;
const int Wheel_Back_Y = 95;
const int Body_X = 96;
const int Body_Y = 30;
const int Body_Width = 66;
const int Body_Height = 90;
const int Joystick_Centre_X = 129;
const int Joystick_Centre_Y = 84;
const int Joystick_Radius = 30;
const int Divider_X = 245;
const int Speed_Gauge_X = 265;
const int Speed_Gauge_Width = 40;
const int Speed_Gauge_Top = 25;
const int Speed_Gauge_Bottom = 145;

// Zone bars around the chair, LiDAR outside and PSD inside at the front
const int Zone_Bar_Thickness = 6;
const int Lidar_Front_Bar_Y = 10;
const int PSD_Front_Bar_Y = 20;
const int PSD_Back_Bar_Y = Body_Y + Body_Height + 4;
const int PSD_Side_Left_Bar_X = 84;
const int PSD_Side_Right_Bar_X = 168;

// PSD screen
const int PSD_Bar_Start_X = 10;
const int PSD_Bar_Spacing = 62;
const int PSD_Bar_Width = 40;
const int PSD_Bar_Bottom = 140;
const int PSD_Bar_Height = 80;

// LiDAR screen, the fan starts at the bottom and opens upward
const int Lidar_Centre_X = 100;
const int Lidar_Centre_Y = 160;
const int Lidar_Panel_X = 208;
const int Lidar_Radar_Radius = 100;

// Wheel outline with diagonal roller lines
static void Draw_Wheel(int X, int Y, bool Forward, uint16_t Color) {
  Display_Sprite.drawRect(X, Y, Wheel_Width, Wheel_Height, Color);
  int Step = Wheel_Height / 3;
  for (int i = 0; i < 3; i++) {
    if (Forward) {
      Display_Sprite.drawLine(X, Y + i * Step, X + Wheel_Width,
        Y + (i + 1) * Step, Color);
    } else {
      Display_Sprite.drawLine(X, Y + (i + 1) * Step, X + Wheel_Width,
        Y + i * Step, Color);
    }
  }
}

// Speed bar next to each wheel, green forward and red reverse
static void Draw_Wheel_Speed(int X, int Y, int Speed) {
  const int Half_Height = 22;
  const int Width = 12;
  int Height = constrain(map(Speed, -Max_Speed, Max_Speed, -Half_Height, Half_Height),
    -Half_Height, Half_Height);
  if (Height < 0) {
    Display_Sprite.fillRect(X, Y - Half_Height, Width, Half_Height, TFT_RED);
    Display_Sprite.fillRect(X, Y - Half_Height, Width, Half_Height + Height, TFT_BLACK);
  } else {
    Display_Sprite.fillRect(X, Y, Width, Height, TFT_GREEN);
  }
  Display_Sprite.drawRect(X, Y - Half_Height, Width, Half_Height * 2, TFT_WHITE);
}

// Speed gauge that turns orange or red and shows SLOW, CRAWL or STOP when collision kicks in
static void Draw_Speed_Gauge(float Speed) {
  Display_Sprite.drawLine(Divider_X, 5, Divider_X, Screen_Height - 5, TFT_DARKGREY);

  const char *Title = "SPEED";
  uint16_t Color = TFT_GREEN;
  switch (Get_Speed_Limit()) {
    case Limit_Slow: Title = "! SLOW"; Color = TFT_ORANGE; break;
    case Limit_Crawl: Title = "! CRAWL"; Color = TFT_RED; break;
    case Limit_Stop: Title = "! STOP"; Color = TFT_RED; break;
    default: break;
  }

  int Centre_X = Speed_Gauge_X + Speed_Gauge_Width / 2;
  Display_Sprite.setTextDatum(TC_DATUM);
  Display_Sprite.setTextColor(Color == TFT_GREEN ? TFT_WHITE : Color);
  Display_Sprite.setTextSize(1);
  Display_Sprite.drawString(Title, Centre_X, 8);

  int Gauge_Height = Speed_Gauge_Bottom - Speed_Gauge_Top;
  int Fill_Height = constrain((int)(Speed * Gauge_Height), 0, Gauge_Height);
  Display_Sprite.drawRect(Speed_Gauge_X, Speed_Gauge_Top, Speed_Gauge_Width, Gauge_Height,
    Color == TFT_GREEN ? TFT_WHITE : Color);
  if (Fill_Height > 0) {
    Display_Sprite.fillRect(Speed_Gauge_X + 1, Speed_Gauge_Bottom - Fill_Height,
      Speed_Gauge_Width - 2, Fill_Height - 1, Color);
  }

  Display_Sprite.setTextSize(2);
  Display_Sprite.drawString(String((int)(Speed * 100)) + "%", Centre_X, Speed_Gauge_Bottom + 8);
}

// Green is clear, orange slow, red crawl and maroon stop
static uint16_t Zone_Bar_Color(CollisionZone Zone) {
  switch (Zone) {
    case Zone_Slow: return TFT_ORANGE;
    case Zone_Crawl: return TFT_RED;
    case Zone_Stop: return TFT_MAROON;
    default: return TFT_GREEN;
  }
}

// Ignored sensors are drawn grey
static void Draw_Zone_Bar(int X, int Y, int Width, int Height, CollisionZone Zone, bool Ignored = false) {
  Display_Sprite.fillRect(X, Y, Width, Height, Ignored ? TFT_DARKGREY : Zone_Bar_Color(Zone));
}

// Draw the chair, wheel speeds, joystick, speed gauge and zone bars
static void Draw_Default_Menu(const ControllerInput &Input, const int Wheels[4]) {
  bool On = Motors_Are_Enabled();
  int Front_Left = On ? Wheels[Wheel_Front_Left] : 0;
  int Front_Right = On ? Wheels[Wheel_Front_Right] : 0;
  int Back_Left = On ? Wheels[Wheel_Back_Left] : 0;
  int Back_Right = On ? Wheels[Wheel_Back_Right] : 0;

  // Robot body and wheels
  Display_Sprite.drawRect(Body_X, Body_Y, Body_Width, Body_Height, TFT_WHITE);
  Draw_Wheel(Wheel_Left_X - Wheel_Width - 8, Wheel_Front_Y, true, TFT_WHITE);
  Draw_Wheel(Wheel_Left_X - Wheel_Width - 8, Wheel_Back_Y, false, TFT_WHITE);
  Draw_Wheel(Wheel_Right_X + 8, Wheel_Front_Y, false, TFT_WHITE);
  Draw_Wheel(Wheel_Right_X + 8, Wheel_Back_Y, true, TFT_WHITE);

  Draw_Wheel_Speed(Wheel_Left_X, Wheel_Front_Y + Wheel_Height / 2, Front_Left);
  Draw_Wheel_Speed(Wheel_Left_X, Wheel_Back_Y + Wheel_Height / 2, Back_Left);
  Draw_Wheel_Speed(Wheel_Right_X, Wheel_Front_Y + Wheel_Height / 2, Front_Right);
  Draw_Wheel_Speed(Wheel_Right_X, Wheel_Back_Y + Wheel_Height / 2, Back_Right);

  // Joystick dot and rotation needle, the needle swings left for positive rotation
  int x2 = -sin((Input.Rotation * PI) / 3) * Joystick_Radius * 0.7f;
  int y2 = -cos((Input.Rotation * PI) / 3) * Joystick_Radius * 0.7f;
  Display_Sprite.drawWideLine(Joystick_Centre_X, Joystick_Centre_Y,
    Joystick_Centre_X + x2, Joystick_Centre_Y + y2, 5, TFT_GREEN);
  Display_Sprite.drawCircle(Joystick_Centre_X, Joystick_Centre_Y, Joystick_Radius, TFT_WHITE);
  Display_Sprite.fillCircle(Joystick_Centre_X + Input.X * (Joystick_Radius - 5),
    Joystick_Centre_Y - Input.Y * (Joystick_Radius - 5), 5, TFT_RED);

  Draw_Speed_Gauge(Input.Speed);

  PSD_Zones PSD = Get_PSD_Zones();
  Lidar_Zones Lidar = Get_Lidar_Zones();
  int Half_Width = Body_Width / 2;
  int Right_Half_X = Body_X + Half_Width;
  int Right_Half_Width = Body_Width - Half_Width;

  Draw_Zone_Bar(Body_X, Lidar_Front_Bar_Y, Half_Width, Zone_Bar_Thickness, Lidar.Front_Left);
  Draw_Zone_Bar(Right_Half_X, Lidar_Front_Bar_Y, Right_Half_Width, Zone_Bar_Thickness, Lidar.Front_Right);
  Draw_Zone_Bar(Body_X, PSD_Front_Bar_Y, Half_Width, Zone_Bar_Thickness, PSD.Front_Left, Ignore_Front_PSD);
  Draw_Zone_Bar(Right_Half_X, PSD_Front_Bar_Y, Right_Half_Width, Zone_Bar_Thickness, PSD.Front_Right, Ignore_Front_PSD);

  Draw_Zone_Bar(Body_X, PSD_Back_Bar_Y, Body_Width, Zone_Bar_Thickness, PSD.Back);
  Draw_Zone_Bar(PSD_Side_Left_Bar_X, Body_Y, Zone_Bar_Thickness, Body_Height, PSD.Side_Left);
  Draw_Zone_Bar(PSD_Side_Right_Bar_X, Body_Y, Zone_Bar_Thickness, Body_Height, PSD.Side_Right);
}

// Colour for a PSD bar based on its level from 0 to 7
static uint16_t Get_Zone_Color(int zone) {
  if (zone <= 2) return TFT_RED;
  else if (zone <= 4) return TFT_ORANGE;
  return TFT_GREEN;
}

// One bar per PSD sensor, a taller bar means more free space
static void Draw_PSD_Menu() {
  Display_Sprite.setTextDatum(TC_DATUM);
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setTextSize(2);
  Display_Sprite.drawString("PSD distances (cm)", Screen_Width / 2, 4);
  Display_Sprite.drawLine(0, 28, Screen_Width - 1, 28, TFT_WHITE);

  PSD_Distances Distances = Get_PSD_Distances();
  int Sensor_Values[5] = {
    Distances.Front_Left,
    Distances.Front_Right,
    Distances.Side_Left,
    Distances.Side_Right,
    Distances.Back
  };
  const char *Sensor_Names[5] = {"FL", "FR", "SL", "SR", "B"};

  Display_Sprite.setTextSize(1);
  for (int i = 0; i < 5; i++) {
    int X = PSD_Bar_Start_X + PSD_Bar_Spacing * i;
    int Centre_X = X + PSD_Bar_Width / 2;
    int Zone = constrain(map(Sensor_Values[i], PSD_Min_Distance, PSD_Max_Distance, 0, 7), 0, 7);
    int Height = map(Zone, 0, 7, 0, PSD_Bar_Height);

    Display_Sprite.setTextColor(TFT_WHITE);
    Display_Sprite.drawString(Sensor_Names[i], Centre_X, 34);
    Display_Sprite.drawRect(X, PSD_Bar_Bottom - PSD_Bar_Height, PSD_Bar_Width, PSD_Bar_Height, TFT_WHITE);
    if (Height > 0) {
      Display_Sprite.fillRect(X + 1, PSD_Bar_Bottom - Height, PSD_Bar_Width - 2, Height - 1,
        Get_Zone_Color(Zone));
    }
    Display_Sprite.drawString(String(Sensor_Values[i]), Centre_X, PSD_Bar_Bottom + 10);
  }
}

// Radar points turn orange inside the LiDAR slow range
static uint16_t Get_Lidar_Point_Color(int Distance) {
  return Distance <= Lidar_Slow_Distance ? TFT_ORANGE : TFT_GREEN;
}

// Screen position on the fan, angle 0 is straight ahead and positive is right
static void Fan_Point(int Angle, int Radius, int &X, int &Y) {
  float Rad = Angle * DEG_TO_RAD;
  X = Lidar_Centre_X + sin(Rad) * Radius;
  Y = Lidar_Centre_Y - cos(Rad) * Radius;
}

// Range ring across the cone, drawn with short lines
static void Draw_Fan_Arc(int Radius, uint16_t Color) {
  const int Half = Lidar_Visible_Angle / 2;
  int X1, Y1, X2, Y2;
  Fan_Point(-Half, Radius, X1, Y1);
  for (int Angle = -Half + 4; Angle <= Half + 3; Angle += 4) {
    Fan_Point(min(Angle, Half), Radius, X2, Y2);
    Display_Sprite.drawLine(X1, Y1, X2, Y2, Color);
    X1 = X2;
    Y1 = Y2;
  }
}

// Front-cone fan with range rings, edge lines and live points
static void Draw_Lidar_Radar() {
  const int Half = Lidar_Visible_Angle / 2;
  LidarDisplayPoint Points[Lidar_Visible_Angle / Lidar_Radar_Point_Step + 1];
  int Point_Count = Get_Lidar_Display_Points(Points, Lidar_Visible_Angle / Lidar_Radar_Point_Step + 1);

  for (int Ring = 1; Ring <= 3; Ring++) {
    Draw_Fan_Arc(Lidar_Radar_Radius * Ring / 3, TFT_DARKGREY);
  }

  int X, Y;
  Fan_Point(0, Lidar_Radar_Radius, X, Y);
  Display_Sprite.drawLine(Lidar_Centre_X, Lidar_Centre_Y, X, Y, TFT_DARKGREY);
  for (int Edge : {-Half, Half}) {
    Fan_Point(Edge, Lidar_Radar_Radius, X, Y);
    Display_Sprite.drawLine(Lidar_Centre_X, Lidar_Centre_Y, X, Y, TFT_YELLOW);
  }

  for (int i = 0; i < Point_Count; i++) {
    int Radius = map(Points[i].Distance, 0, Lidar_Max_Distance, 0, Lidar_Radar_Radius);
    Fan_Point(Points[i].Angle, Radius, X, Y);
    Display_Sprite.fillCircle(X, Y, 2, Get_Lidar_Point_Color(Points[i].Distance));
  }

  Display_Sprite.fillTriangle(Lidar_Centre_X, Lidar_Centre_Y - 7,
    Lidar_Centre_X - 5, Lidar_Centre_Y + 5, Lidar_Centre_X + 5, Lidar_Centre_Y + 5, TFT_WHITE);

  Display_Sprite.setTextDatum(MC_DATUM);
  Display_Sprite.setTextSize(1);
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.drawString("F", Lidar_Centre_X, Lidar_Centre_Y - Lidar_Radar_Radius - 8);
}

// Title and line at the top of a text screen
static void Draw_Title(const char *Title) {
  Display_Sprite.setTextDatum(TL_DATUM);
  Display_Sprite.setTextSize(2);
  Display_Sprite.setTextColor(TFT_RED);
  Display_Sprite.setCursor(5, 4);
  Display_Sprite.print(Title);
  Display_Sprite.drawLine(0, 28, Screen_Width - 1, 28, TFT_WHITE);
  Display_Sprite.setTextSize(1);
}

// Label and distance, coloured by the zone collision avoidance is using
static void Draw_Lidar_Zone_Row(const char *Label, int Distance, CollisionZone Zone, int X, int Y) {
  Display_Sprite.setTextDatum(TL_DATUM);
  Display_Sprite.setTextSize(1);
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.drawString(Label, X, Y);
  Display_Sprite.setTextSize(2);
  Display_Sprite.setTextColor(Zone_Bar_Color(Zone));
  Display_Sprite.drawString(Distance > 0 ? String(Distance) + "cm" : "--", X, Y + 12);
}

// Radar fan with status and the closest distance on each front half
static void Draw_LiDAR_Menu() {
  LidarData Data = Get_Lidar_Data();
  Lidar_Zones Zones = Get_Lidar_Zones();

  Draw_Title("LIDAR");
  Draw_Lidar_Radar();

  int Panel_X = Lidar_Panel_X;
  Display_Sprite.setTextSize(1);
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.drawString("Status:", Panel_X, 36);
  Display_Sprite.setTextColor(Data.Connected ? TFT_GREEN : TFT_RED);
  Display_Sprite.drawString(Data.Connected ? "OK" : "OFF", Panel_X + 55, 36);

  Draw_Lidar_Zone_Row("Front-Left", Data.Front_Left, Zones.Front_Left, Panel_X, 56);
  Draw_Lidar_Zone_Row("Front-Right", Data.Front_Right, Zones.Front_Right, Panel_X, 94);

  if (Data.Closest > 0 && Data.Closest <= Lidar_Slow_Distance) {
    Display_Sprite.setTextColor(TFT_ORANGE);
    Display_Sprite.setTextSize(2);
    Display_Sprite.drawString("SLOWING", Panel_X, 140);
  }
}

// One label and value row on a text screen
static void Draw_Row(const char *Label, const String &Value, int Y, uint16_t Value_Color = TFT_WHITE) {
  const int Label_X = 10;
  const int Value_X = 180;
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setCursor(Label_X, Y);
  Display_Sprite.print(Label);
  Display_Sprite.setTextColor(Value_Color);
  Display_Sprite.setCursor(Value_X, Y);
  Display_Sprite.print(Value);
}

static String On_Off(bool Value) {
  return Value ? "ON" : "OFF";
}

// Minutes and seconds, like 2:05
static String Clock_Text(unsigned long Seconds) {
  String Text = String(Seconds / 60) + ":";
  if (Seconds % 60 < 10) Text += "0";
  return Text + String(Seconds % 60);
}

// Settings you can toggle with buttons 7-12
static void Draw_Config_Menu() {
  Draw_Title("Config");
  int Y = 34;
  const int Row_Height = 18;

  Draw_Row("Collision avoidance (btn 7)", On_Off(Collision_Enabled), Y);
  Draw_Row("Hardstop (btn 8)", On_Off(Hardstop_Enabled), Y += Row_Height);
  Draw_Row("Square input (btn 9)", On_Off(Square_Inputs), Y += Row_Height);
  Draw_Row("Ignore front PSD (btn 10)", On_Off(Ignore_Front_PSD), Y += Row_Height);
  Draw_Row("Web UI (btn 11)", On_Off(Web_UI_Enabled), Y += Row_Height);
  if (Is_Recording()) Draw_Row("Recording (btn 12)", "REC " + Clock_Text(Recorded_Seconds()), Y += Row_Height, TFT_RED);
  else Draw_Row("Recording (btn 12)", "OFF", Y += Row_Height);
  String WiFi_Text = Web_UI_Enabled ? String(AP_Name) + " (" + WiFi.softAPIP().toString() + ")" : "OFF";
  Draw_Row("WiFi", WiFi_Text, Y += Row_Height);
}

// Loop rate, sensor rates, memory and uptime
static void Draw_Stats_Menu() {
  Draw_Title("Stats");
  SystemStats Stats = Get_Stats();
  int Y = 34;
  const int Row_Height = 16;

  Draw_Row("Uptime", Clock_Text(Stats.Uptime), Y);
  Draw_Row("Loop rate", String(Stats.Loop_Rate) + " /s", Y += Row_Height);
  Draw_Row("LiDAR", String(Stats.Lidar_Rate, 1) + " rotations/s", Y += Row_Height);
  Draw_Row("LiDAR bad packets", String(Stats.Lidar_Bad_Packets), Y += Row_Height);
  Draw_Row("Joystick", String(Stats.Joystick_Rate) + " reports/s", Y += Row_Height);
  Draw_Row("Free memory", String(Stats.Free_Heap / 1024) + " KB", Y += Row_Height);
  Draw_Row("Free PSRAM", String(Stats.Free_PSRAM / 1024) + " KB", Y += Row_Height);
  Draw_Row("WiFi clients", String(Stats.WiFi_Clients), Y += Row_Height);
  Draw_Row("Last reset", Stats.Reset_Reason, Y += Row_Height);
}

// A PSD passes if its raw reading is inside what the sensor can actually output
static void Draw_PSD_Test_Row(const char *Label, int Raw, int Distance, int Y) {
  bool OK = Raw >= 50 && Raw <= 1000;
  Draw_Row(Label, String(OK ? "OK " : "FAIL ") + String(Distance) + " cm (raw " + String(Raw) + ")", Y, OK ? TFT_GREEN : TFT_RED);
}

void Draw_Self_Test() {
  if (millis() - Display_Update_Timer < Display_Update_Time) return;
  Display_Update_Timer = millis();

  Display_Sprite.fillSprite(TFT_BLACK);
  Draw_Title("Self test");
  int Y = 32;
  const int Row_Height = 13;

  PSD_Distances Raw = Get_PSD_Raw();
  PSD_Distances Distance = Get_PSD_Distances();
  bool Shield = USB_Host_Is_Ready();
  bool Joystick = USB_Controller_Is_Valid();
  bool Lidar = Get_Lidar_Data().Connected;

  Draw_Row("USB Host Shield", Shield ? "OK" : "FAIL", Y, Shield ? TFT_GREEN : TFT_RED);
  Draw_Row("Joystick", Joystick ? "OK" : "WAITING", Y += Row_Height, Joystick ? TFT_GREEN : TFT_ORANGE);
  Draw_Row("LiDAR", Lidar ? "OK" : "WAITING", Y += Row_Height, Lidar ? TFT_GREEN : TFT_ORANGE);
  Draw_PSD_Test_Row("PSD front left", Raw.Front_Left, Distance.Front_Left, Y += Row_Height);
  Draw_PSD_Test_Row("PSD front right", Raw.Front_Right, Distance.Front_Right, Y += Row_Height);
  Draw_PSD_Test_Row("PSD side left", Raw.Side_Left, Distance.Side_Left, Y += Row_Height);
  Draw_PSD_Test_Row("PSD side right", Raw.Side_Right, Distance.Side_Right, Y += Row_Height);
  Draw_PSD_Test_Row("PSD back", Raw.Back, Distance.Back, Y += Row_Height);
  Draw_Row("WiFi", Web_UI_Enabled ? "ON" : "OFF", Y += Row_Height);
  Draw_Row("Settings", Settings_Were_Saved() ? "loaded" : "defaults", Y += Row_Height);

  Display_Sprite.pushSprite(0, 0);
}

// Start the screen and show the boot message
void Setup_Display() {
  if (TFT_BL > 0) {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  }

  TFT.init();
  TFT.setRotation(3);
  TFT.fillScreen(TFT_BLACK);
  TFT.setSwapBytes(true);

  Display_Sprite.setColorDepth(8);
  Display_Sprite.createSprite(Screen_Width, Screen_Height);
  Display_Sprite.fillSprite(TFT_BLACK);
  Display_Sprite.setTextColor(TFT_GREEN);
  Display_Sprite.setTextSize(2);
  Display_Sprite.setCursor(10, 20);
  Display_Sprite.println("OMNI Chair");
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setTextSize(1);
  Display_Sprite.setCursor(10, 60);
  Display_Sprite.println("Boot OK");
  Display_Sprite.pushSprite(0, 0);
  delay(1000);
}

// Redraw the current screen every Display_Update_Time ms
void Update_Display(const ControllerInput &Input, const int Wheels[4]) {
  if (millis() - Display_Update_Timer < Display_Update_Time) return;
  Display_Update_Timer = millis();

  Display_Sprite.fillSprite(TFT_BLACK);

  if (Input.Menu == PSD_Info_Menu) {
    Draw_PSD_Menu();
  } else if (Input.Menu == LiDAR_Info_Menu) {
    Draw_LiDAR_Menu();
  } else if (Input.Menu == Config_Menu) {
    Draw_Config_Menu();
  } else if (Input.Menu == Stats_Menu) {
    Draw_Stats_Menu();
  } else {
    Draw_Default_Menu(Input, Wheels);
  }

  Display_Sprite.pushSprite(0, 0);
}
