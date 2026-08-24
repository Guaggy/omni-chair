#include "display.h"

#include <Arduino.h>
#include <TFT_eSPI.h>

static TFT_eSPI TFT(135, 240);

const int Wheel_Left_X = 64;
const int Wheel_Right_X = 155;
const int Wheel_Front_Y = 14;
const int Wheel_Back_Y = 75;
const int Wheel_Width = 23;
const int Wheel_Height = 43;

static void Draw_Wheel(int X, int Y, bool Direction, uint16_t Color) {
  TFT.drawRect(X, Y, Wheel_Width, Wheel_Height, Color);
  int Step = Wheel_Height / 3;

  for (int i = 0; i < 3; i++) {
    if (Direction) {
      TFT.drawLine(X, Y + i * Step, X + Wheel_Width, Y + (i + 1) * Step, Color);
    } else {
      TFT.drawLine(X, Y + (i + 1) * Step, X + Wheel_Width, Y + i * Step, Color);
    }
  }
}

// Set up the TFT and draw the wheelchair outline
void Setup_Display() {
  TFT.init();
  TFT.setRotation(3);
  TFT.fillScreen(TFT_BLACK);
  TFT.setTextColor(TFT_WHITE);
  TFT.setSwapBytes(true);

  if (TFT_BL > 0) {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  }

  TFT.drawRect(93, 28, 55, 77, TFT_WHITE);
  TFT.drawCircle(120, 67, 27, TFT_WHITE);
  Draw_Wheel(Wheel_Left_X, Wheel_Front_Y, true, TFT_WHITE);
  Draw_Wheel(Wheel_Left_X, Wheel_Back_Y, false, TFT_WHITE);
  Draw_Wheel(Wheel_Right_X, Wheel_Front_Y, false, TFT_WHITE);
  Draw_Wheel(Wheel_Right_X, Wheel_Back_Y, true, TFT_WHITE);
}

// Update joystick position and motor values on the TFT
void Update_Display(const ControllerInput &Input, int Front_Left, int Front_Right, int Back_Left, int Back_Right) {
  static unsigned long Update_Timer = 0;
  if (millis() - Update_Timer < 200) return;
  Update_Timer = millis();

  // PSD info menu
  if (Input.Menu == PSD_Info_Menu) {
    TFT.fillRect(94, 29, 53, 75, TFT_BLACK);
    TFT.setCursor(5, 5);
    TFT.setTextSize(3);
    TFT.print("Testing PSD menu");
  }
  
  // Default menu
  else {
    TFT.fillRect(94, 29, 53, 75, TFT_BLACK);
    TFT.drawCircle(120, 67, 27, TFT_WHITE);
    TFT.fillCircle(120 + Input.X * 17, 67 - Input.Y * 17, 5, TFT_RED);

    TFT.fillRect(0, 0, 46, 135, TFT_BLACK);
    TFT.setCursor(0, 25);
    TFT.print(Front_Left);
    TFT.setCursor(0, 90);
    TFT.print(Back_Left);

    TFT.fillRect(185, 0, 55, 135, TFT_BLACK);
    TFT.setCursor(185, 25);
    TFT.print(Front_Right);
    TFT.setCursor(185, 90);
    TFT.print(Back_Right);
  }
}

// Colour blocked wheels and show the collision warning
void Update_Collision_Display(bool Collision_Forward, bool Collision_Left,
  bool Collision_Right, bool Collision_Back) {
  Draw_Wheel(Wheel_Left_X, Wheel_Front_Y, true,
    Collision_Forward || Collision_Left ? TFT_RED : TFT_WHITE);
  Draw_Wheel(Wheel_Right_X, Wheel_Front_Y, false,
    Collision_Forward || Collision_Right ? TFT_RED : TFT_WHITE);
  Draw_Wheel(Wheel_Left_X, Wheel_Back_Y, false,
    Collision_Left || Collision_Back ? TFT_RED : TFT_WHITE);
  Draw_Wheel(Wheel_Right_X, Wheel_Back_Y, true,
    Collision_Right || Collision_Back ? TFT_RED : TFT_WHITE);

  if (Collision_Forward || Collision_Left || Collision_Right || Collision_Back) {
    TFT.setTextSize(2);
    TFT.setTextDatum(MC_DATUM);
    TFT.setTextColor(TFT_RED);
    TFT.drawString("COLLISION ON!", 120, 120);
  } else {
    TFT.fillRect(40, 110, 160, 25, TFT_BLACK);
  }
  TFT.setTextColor(TFT_WHITE);
}
