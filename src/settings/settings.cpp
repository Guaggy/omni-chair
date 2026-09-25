#include "settings.h"

#include <Preferences.h>
#include "configs/config.h"

// Stored in the ESP32's flash (NVS), which spreads out the writes so it doesn't wear
static Preferences Store;
static bool Found_Saved = false;

void Load_Settings() {
  Store.begin("omnichair", false);
  Found_Saved = Store.isKey("hardstop");
  Hardstop_Enabled = Store.getBool("hardstop", Hardstop_Enabled);
  Square_Inputs = Store.getBool("square", Square_Inputs);
  Ignore_Front_PSD = Store.getBool("ignore_psd", Ignore_Front_PSD);
  Web_UI_Enabled = Store.getBool("web_ui", Web_UI_Enabled);
  Debug_PSD = Store.getBool("dbg_psd", Debug_PSD);
  Debug_Joystick = Store.getBool("dbg_joy", Debug_Joystick);
  Debug_Joystick_Raw = Store.getBool("dbg_joy_raw", Debug_Joystick_Raw);
  Debug_Lidar = Store.getBool("dbg_lidar", Debug_Lidar);
  Debug_Collision = Store.getBool("dbg_coll", Debug_Collision);
  Debug_Motors = Store.getBool("dbg_motors", Debug_Motors);
  Store.end();
}

void Save_Settings() {
  Store.begin("omnichair", false);
  Store.putBool("hardstop", Hardstop_Enabled);
  Store.putBool("square", Square_Inputs);
  Store.putBool("ignore_psd", Ignore_Front_PSD);
  Store.putBool("web_ui", Web_UI_Enabled);
  Store.putBool("dbg_psd", Debug_PSD);
  Store.putBool("dbg_joy", Debug_Joystick);
  Store.putBool("dbg_joy_raw", Debug_Joystick_Raw);
  Store.putBool("dbg_lidar", Debug_Lidar);
  Store.putBool("dbg_coll", Debug_Collision);
  Store.putBool("dbg_motors", Debug_Motors);
  Store.end();
}

bool Settings_Were_Saved() {
  return Found_Saved;
}
