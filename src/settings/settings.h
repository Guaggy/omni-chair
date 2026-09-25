#pragma once

// Load the saved settings, anything not saved yet keeps its config.h value
void Load_Settings();

// Save the settings that carry over to the next boot, collision is left out on purpose
void Save_Settings();

// True if settings were found in flash at boot
bool Settings_Were_Saved();
