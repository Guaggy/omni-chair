#include "display.h"

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "config.h"
#include "controller.h"
#include "safety/safety.h"
#include "sensors/collision_sensors.h"
#include "sensors/lidar_sensor.h"

static TFT_eSPI TFT(135, 240);
static TFT_eSprite Display_Sprite(&TFT);
static unsigned long Display_Update_Timer = 0;

static bool Display_Collision_Forward = false;
static bool Display_Collision_Left = false;
static bool Display_Collision_Right = false;
static bool Display_Collision_Back = false;

const int Wheel_Left_X = 64;
const int Wheel_Right_X = 155;
const int Wheel_Front_Y = 14;
const int Wheel_Back_Y = 75;
const int Wheel_Width = 23;
const int Wheel_Height = 43;

static void Draw_Wheel(int X, int Y, bool Forward, uint16_t Color) {
  // draw wheel
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

static void Draw_Speed(int X, int Y, int speed) {
  int height = constrain(map(speed, -Max_Speed, Max_Speed, -20, 20), -20, 20);
  if (height < 0) {
    Display_Sprite.fillRect(X, Y-20, 10, 20, TFT_RED);
    Display_Sprite.fillRect(X, Y-20, 10, 20+height, TFT_BLACK);
  }
  else {
    Display_Sprite.fillRect(X, Y, 10, height, TFT_GREEN);
  }
  Display_Sprite.drawRect(X, Y-20, 10, 40, TFT_WHITE);
}

// Draw the normal driving screen
static void Draw_Default_Menu(const ControllerInput &Input, int Front_Left, int Front_Right, int Back_Left, int Back_Right) {
  if (!Motors_Are_Enabled()) {
    Front_Left = 0;
    Front_Right = 0;
    Back_Left = 0;
    Back_Right = 0;
  }

  uint16_t Front_Left_Color = Display_Collision_Forward || Display_Collision_Left ? TFT_RED : TFT_WHITE;
  uint16_t Front_Right_Color = Display_Collision_Forward || Display_Collision_Right ? TFT_RED : TFT_WHITE;
  uint16_t Back_Left_Color = Display_Collision_Left || Display_Collision_Back ? TFT_RED : TFT_WHITE;
  uint16_t Back_Right_Color = Display_Collision_Right || Display_Collision_Back ? TFT_RED : TFT_WHITE;

  Display_Sprite.setTextDatum(TL_DATUM);
  Display_Sprite.setTextColor(TFT_WHITE);

  // Robot body and wheels
  Display_Sprite.drawRect(93, 28, 55, 77, TFT_WHITE);
  Draw_Wheel(Wheel_Left_X - 30, Wheel_Front_Y, true, Front_Left_Color);
  Draw_Wheel(Wheel_Left_X - 30, Wheel_Back_Y, false, Back_Left_Color);
  Draw_Wheel(Wheel_Right_X + 20, Wheel_Front_Y, false, Front_Right_Color);
  Draw_Wheel(Wheel_Right_X + 20, Wheel_Back_Y, true, Back_Right_Color);

  Draw_Speed(Wheel_Left_X, Wheel_Front_Y + Wheel_Height / 2, Front_Left);
  Draw_Speed(Wheel_Left_X, Wheel_Back_Y + Wheel_Height / 2, Back_Left);
  Draw_Speed(Wheel_Right_X, Wheel_Front_Y + Wheel_Height / 2, Front_Right);
  Draw_Speed(Wheel_Right_X, Wheel_Back_Y + Wheel_Height / 2, Back_Right);

  // Joystick position and rotation
  int x2 = sin((Input.Rotation * PI)/3) * 20;
  int y2 = -cos((Input.Rotation * PI)/3) * 20;
  Display_Sprite.drawWideLine(120, 67, 120 + x2, 67 + y2, 5, TFT_GREEN);
  Display_Sprite.drawCircle(120, 67, 27, TFT_WHITE);
  Display_Sprite.fillCircle(120 + Input.X * 17, 67 - Input.Y * 17, 5, TFT_RED);

  if (Display_Collision_Forward || Display_Collision_Left || Display_Collision_Right || Display_Collision_Back) {
    Display_Sprite.setTextDatum(MC_DATUM);
    Display_Sprite.setTextColor(TFT_RED);
    Display_Sprite.setTextSize(1);
    Display_Sprite.drawString("!!! COLLISION !!!", 120, 125);
  }
}

// Return colors for PSD display
static uint16_t Get_Zone_Color(int zone) {
  if (zone <= 2) return TFT_RED;
  else if (zone <= 4) return TFT_ORANGE;
  return TFT_GREEN;
}

// Show the latest stored PSD distances
static void Draw_PSD_Menu() {
  Display_Sprite.setTextDatum(TC_DATUM);
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setTextSize(2);
  Display_Sprite.setCursor(120, 0);
  Display_Sprite.print("PSD distances (cm)");
  Display_Sprite.drawLine(0, 30, 239, 30, TFT_WHITE);

  PSD_Distances Distances = Get_PSD_Distances();
  int Sensor_Values[5] = {
    Distances.Front_Left,
    Distances.Front_Right,
    Distances.Side_Left,
    Distances.Side_Right,
    Distances.Back
  };
  const char *Sensor_Names[5] = {"FL", "FR", "SL", "SR", "B"};
  const int Bar_Bottom = 105;
  const int Bar_Height = 55;
  const int Bar_Width = 28;

  Display_Sprite.setTextSize(1);
  for (int i = 0; i < 5; i++) {
    int X = 10 + 46 * i;
    int Zone = constrain(map(Sensor_Values[i], sensor_min, sensor_max, 0, 7), 0, 7);
    int Height = map(Zone, 0, 7, 0, Bar_Height);

    Display_Sprite.setCursor(X + 23, 36);
    Display_Sprite.print(Sensor_Names[i]);
    Display_Sprite.drawRect(X, Bar_Bottom - Bar_Height, Bar_Width, Bar_Height, TFT_WHITE);
    if (Height > 0) {
      Display_Sprite.fillRect(X + 1, Bar_Bottom - Height, Bar_Width - 2, Height - 1,
        Get_Zone_Color(Zone));
    }
    Display_Sprite.setCursor(X + 23, 114);
    Display_Sprite.print(Sensor_Values[i]);
  }
}

static uint16_t Get_Lidar_Point_Color(int Distance) {
  if (Distance <= Lidar_Stop_Distance) return TFT_RED;
  if (Distance <= Lidar_Warning_Distance) return TFT_ORANGE;
  return TFT_GREEN;
}

static const char *Get_Closest_Lidar_Zone(const LidarData &Data) {
  if (Data.Closest == 0) return "--";
  if (Data.Front == Data.Closest) return "FRONT";
  if (Data.Left == Data.Closest) return "LEFT";
  if (Data.Right == Data.Closest) return "RIGHT";
  return "BACK";
}

// Draw a small radar using one point for every configured angle step
static void Draw_Lidar_Radar() {
  const int Centre_X = 65;
  const int Centre_Y = 82;
  LidarDisplayPoint Points[360 / Lidar_Radar_Point_Step];
  int Point_Count = Get_Lidar_Display_Points(Points, 360 / Lidar_Radar_Point_Step);

  Display_Sprite.drawCircle(Centre_X, Centre_Y, Lidar_Radar_Radius / 3, TFT_DARKGREY);
  Display_Sprite.drawCircle(Centre_X, Centre_Y, Lidar_Radar_Radius * 2 / 3, TFT_DARKGREY);
  Display_Sprite.drawCircle(Centre_X, Centre_Y, Lidar_Radar_Radius, TFT_DARKGREY);
  Display_Sprite.drawLine(Centre_X - Lidar_Radar_Radius, Centre_Y,
    Centre_X + Lidar_Radar_Radius, Centre_Y, TFT_DARKGREY);
  Display_Sprite.drawLine(Centre_X, Centre_Y - Lidar_Radar_Radius,
    Centre_X, Centre_Y + Lidar_Radar_Radius, TFT_DARKGREY);

  // Convert LiDAR angle and distance to a position inside the radar
  for (int i = 0; i < Point_Count; i++) {
    int Radius = map(Points[i].Distance, 0, Lidar_Max_Distance, 0, Lidar_Radar_Radius);
    float Angle = Points[i].Angle * DEG_TO_RAD;
    int X = Centre_X + sin(Angle) * Radius;
    int Y = Centre_Y - cos(Angle) * Radius;
    Display_Sprite.fillCircle(X, Y, 2, Get_Lidar_Point_Color(Points[i].Distance));
  }

  // Simple chair marker pointing toward the front of the radar
  Display_Sprite.fillTriangle(Centre_X, Centre_Y - 7,
    Centre_X - 5, Centre_Y + 5, Centre_X + 5, Centre_Y + 5, TFT_WHITE);
  Display_Sprite.setTextSize(1);
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setCursor(Centre_X - 3, 32);
  Display_Sprite.print("F");
  Display_Sprite.setCursor(Centre_X - 3, 126);
  Display_Sprite.print("B");
  Display_Sprite.setCursor(14, Centre_Y - 4);
  Display_Sprite.print("L");
  Display_Sprite.setCursor(112, Centre_Y - 4);
  Display_Sprite.print("R");
}

// Show LiDAR radar, connection state and closest obstacle
static void Draw_LiDAR_Menu() {
  LidarData Data = Get_Lidar_Data();

  Display_Sprite.setTextDatum(TL_DATUM);
  Display_Sprite.setTextSize(2);
  Display_Sprite.setTextColor(TFT_RED);
  Display_Sprite.setCursor(5, 5);
  Display_Sprite.print("LIDAR");
  Display_Sprite.drawLine(0, 30, 239, 30, TFT_WHITE);

  Draw_Lidar_Radar();

  Display_Sprite.setTextSize(1);
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setCursor(135, 38);
  Display_Sprite.print("Status:");
  Display_Sprite.setTextColor(Data.Connected ? TFT_GREEN : TFT_RED);
  Display_Sprite.setCursor(185, 38);
  Display_Sprite.print(Data.Connected ? "OK" : "OFF");

  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setCursor(135, 58);
  Display_Sprite.print("Closest:");
  Display_Sprite.setTextSize(2);
  Display_Sprite.setCursor(135, 70);
  Display_Sprite.print(Data.Closest);
  Display_Sprite.print(" cm");

  Display_Sprite.setTextSize(1);
  Display_Sprite.setCursor(135, 94);
  Display_Sprite.print("Zone: ");
  Display_Sprite.print(Get_Closest_Lidar_Zone(Data));

  if (Data.Closest > 0 && Data.Closest <= Lidar_Warning_Distance) {
    Display_Sprite.setTextColor(TFT_RED);
    Display_Sprite.setTextSize(2);
    Display_Sprite.setCursor(135, 110);
    Display_Sprite.print(Data.Closest <= Lidar_Stop_Distance ? "STOP" : "WARNING");
  }
}

// Draw the existing runtime variable screen
static void Draw_Variable_Menu() {
  Display_Sprite.setTextDatum(TL_DATUM);
  Display_Sprite.setTextSize(2);
  Display_Sprite.setTextColor(TFT_RED);
  Display_Sprite.setCursor(5, 5);
  Display_Sprite.print("Variables");
  Display_Sprite.drawLine(0, 30, 239, 30, TFT_WHITE);

  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setCursor(5, 35);
  Display_Sprite.print("Collision");
  Display_Sprite.setCursor(180, 35);
  Display_Sprite.print(Collision_Enabled ? "ON" : "OFF");
  Display_Sprite.setCursor(5, 60);
  Display_Sprite.print("Motor");
  Display_Sprite.setCursor(180, 60);
  Display_Sprite.print(Motors_Are_Enabled() ? "ON" : "OFF");
  Display_Sprite.setCursor(5, 85);
  Display_Sprite.print("Controller ");
  Display_Sprite.setCursor(180, 85);
  ControllerMode Mode = Get_Controller_Mode();
  if (Mode == Controller_PS3) Display_Sprite.print("PS3");
  else if (Mode == Controller_Bluetooth) Display_Sprite.print("BT");
  else Display_Sprite.print("USB");
}

// Set up the TFT and full-screen sprite
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
  Display_Sprite.createSprite(240, 135);
  Display_Sprite.fillSprite(TFT_BLACK);
  Display_Sprite.setTextColor(TFT_GREEN);
  Display_Sprite.setTextSize(2);
  Display_Sprite.setCursor(10, 20);
  Display_Sprite.println("OMNI Chair V3");
  Display_Sprite.setTextColor(TFT_WHITE);
  Display_Sprite.setTextSize(1);
  Display_Sprite.setCursor(10, 60);
  Display_Sprite.println("Boot OK");
  Display_Sprite.pushSprite(0, 0);
  delay(1000);
}

// Clear, draw and push one complete menu frame
void Update_Display(const ControllerInput &Input, int Front_Left, int Front_Right,
  int Back_Left, int Back_Right) {
  if (millis() - Display_Update_Timer < Display_Update_Time) return;
  Display_Update_Timer = millis();

  Display_Sprite.fillSprite(TFT_BLACK);

  if (Input.Menu == PSD_Info_Menu) {
    Draw_PSD_Menu();
  } else if (Input.Menu == LiDAR_Info_Menu) {
    Draw_LiDAR_Menu();
  } else if (Input.Menu == Variable_Menu) {
    Draw_Variable_Menu();
  } else {
    Draw_Default_Menu(Input, Front_Left, Front_Right, Back_Left, Back_Right);
  }

  Display_Sprite.pushSprite(0, 0);
}

// Save collision state for the next complete display frame
void Update_Collision_Display(bool Collision_Forward, bool Collision_Left,
  bool Collision_Right, bool Collision_Back) {
  Display_Collision_Forward = Collision_Forward;
  Display_Collision_Left = Collision_Left;
  Display_Collision_Right = Collision_Right;
  Display_Collision_Back = Collision_Back;
}
