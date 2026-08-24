#include "collision_sensors.h"

#include <Arduino.h>
#include "config.h"
#include "pins.h"
#include "display/display.h"
#include "safety/safety.h"

bool Collision_Enabled = false;

// Variables
const int sensor_max = 80;
const int sensor_min = 10;

// Filtering
const int sensor_average_num = 5; // number of measurements
const int sensor_remove_spike_minmax_num = 1; // number of highest/lowest values to remove
int sensor_measurements[5][sensor_average_num];
int sensor_measurement_count = 0;
float sensor_average[5];
float final_distance[5];
static unsigned long Last_Sensor_Read = 0;

static void Calculate_Filtered_Average();

const int Sensor_Pins[5] = {
  Sensor_Front_Right_Pin,
  Sensor_Front_Left_Pin,
  Sensor_Side_Right_Pin,
  Sensor_Side_Left_Pin,
  Sensor_Back_Pin
};

// Convert a 10-bit ADC reading using the GP2Y0A21YK0F formula
static int Read_Sensor(int Pin) {
  int ADC_Value = analogRead(Pin);
  int Divisor = ADC_Value - 20;

  if (Divisor <= 0) return sensor_max;

  int Distance = 4800 / Divisor;
  if (Distance > sensor_max) Distance = sensor_max;
  if (Distance < sensor_min) Distance = sensor_min;
  return Distance;
}

// Read values from PSD sensors
void Read_Sensors() {
  if (millis() - Last_Sensor_Read < 20) return;
  Last_Sensor_Read = millis();

  for (int i = 0; i < 5; i++) {
    sensor_measurements[i][sensor_measurement_count] = Read_Sensor(Sensor_Pins[i]);
  }
  sensor_measurement_count++;

  // SPIKE FILTER + AVERAGE + LOWPASS
  if (sensor_measurement_count >= sensor_average_num) {

    // Sort values, remove highest/lowest and average the remaining values
    Calculate_Filtered_Average();

    // LOW PASS FILTER
    for (int i = 0; i < 5; i++) {
      final_distance[i] = final_distance[i] * 0.7f + sensor_average[i] * 0.3f;
      
      // Serial output
      if (Debug_PSD) {
        Serial.println();
        Serial.print("--- Spike filtered average: ");
        Serial.print(sensor_average[i], 1);
        Serial.println(" cm");

        Serial.print("--- Low-pass filtered distance: ");
        Serial.print(final_distance[i], 1);
        Serial.println(" cm");
      }
    }

    // Reset measurement buffer
    sensor_measurement_count = 0;
  }
}

void Setup_Collision_Sensors() {
  // initialize lists
  for (int i = 0; i < 5; i++) {
    final_distance[i] = sensor_max;
  } 
  for (int i = 0; i < 5; i++) {
    sensor_average[i] = 0.0;
  } 
  // SharpIR library uses 10-bit ADC values
  analogReadResolution(10);
}

// Sort measurements, remove high/low spikes and calculate average
static void Calculate_Filtered_Average() {

  // Sort values from lowest to highest
  for (int s = 0; s < 5; s ++) {
    for (int i = 0; i < sensor_average_num - 1; i++) {
      for (int j = 0; j < sensor_average_num - i - 1; j++) {
        
        if (sensor_measurements[s][j] > sensor_measurements[s][j + 1]) {
          int temp = sensor_measurements[s][j];
          sensor_measurements[s][j] = sensor_measurements[s][j + 1];
          sensor_measurements[s][j + 1] = temp;
        }
      }
    }
  }

  // print to serial for debugging
  if (Debug_PSD) {
    for (int s = 0; s < 5; s++) {
      for (int i = 0; i < sensor_average_num; i++) {
        if (i < sensor_remove_spike_minmax_num) {
          Serial.print("(");
          Serial.print(sensor_measurements[s][i]);
          Serial.print("), ");
        }
        else if (i > sensor_average_num - 1 - sensor_remove_spike_minmax_num) {
          Serial.print("(");
          Serial.print(sensor_measurements[s][i]);
          if (i <= sensor_average_num - 1)
          Serial.print(")");
        }
        else {
          Serial.print(sensor_measurements[s][i]);
          Serial.print(", ");
        }
      }
    }
  }

  // Remove specified number of lowest and highest values
  int start_index = sensor_remove_spike_minmax_num;
  int end_index = sensor_average_num - sensor_remove_spike_minmax_num;

  // Sum remaining measurements
  int sum[5];
  for (int s = 0; s < 5; s ++) {
    int temp_var = 0;
    for (int i = start_index; i < end_index; i++) {
      temp_var += sensor_measurements[s][i];
    }
    sum[s] = temp_var;
  }

  // Number of remaining measurements
  int values_remaining = sensor_average_num - 2 * sensor_remove_spike_minmax_num;

  // Calculate average
  for (int i = 0; i < 5; i++) {
    sensor_average[i] = float(sum[i]) / float(values_remaining);
  }
}

// Return the latest filtered distances for collision checks and display
PSD_Distances Get_PSD_Distances() {
  PSD_Distances Distances;
  Distances.Front_Right = final_distance[0];
  Distances.Front_Left = final_distance[1];
  Distances.Side_Right = final_distance[2];
  Distances.Side_Left = final_distance[3];
  Distances.Back = final_distance[4];
  return Distances;
}

// Stop wheel commands that would move toward an obstacle
void Check_Collisions(int &Front_Left, int &Front_Right, int &Back_Left, int &Back_Right) {
  PSD_Distances Distances = Get_PSD_Distances();

  bool Collision_Forward = false;
  bool Collision_Left = false;
  bool Collision_Right = false;
  bool Collision_Back = false;

  if (Motors_Are_Enabled() && Collision_Enabled) {
    if (Distances.Front_Left < Front_Stop_Distance || Distances.Front_Right < Front_Stop_Distance) {
      Collision_Forward = true;
      if (Front_Left > 0) Front_Left = 0;
      if (Front_Right > 0) Front_Right = 0;
      if (Back_Left > 0) Back_Left = 0;
      if (Back_Right > 0) Back_Right = 0;
    }
    if (Distances.Side_Left < Side_Stop_Distance) {
      Collision_Left = true;
      if (Front_Left < 0) Front_Left = 0;
      if (Front_Right > 0) Front_Right = 0;
      if (Back_Left > 0) Back_Left = 0;
      if (Back_Right < 0) Back_Right = 0;
    }
    if (Distances.Side_Right < Side_Stop_Distance) {
      Collision_Right = true;
      if (Front_Left > 0) Front_Left = 0;
      if (Front_Right < 0) Front_Right = 0;
      if (Back_Left < 0) Back_Left = 0;
      if (Back_Right > 0) Back_Right = 0;
    }
    if (Distances.Back < Back_Stop_Distance) {
      Collision_Back = true;
      if (Front_Left < 0) Front_Left = 0;
      if (Front_Right < 0) Front_Right = 0;
      if (Back_Left < 0) Back_Left = 0;
      if (Back_Right < 0) Back_Right = 0;
    }
  }

  static bool Old_Collision_Forward = false;
  static bool Old_Collision_Left = false;
  static bool Old_Collision_Right = false;
  static bool Old_Collision_Back = false;

  bool Unchanged = Collision_Forward == Old_Collision_Forward &&
    Collision_Left == Old_Collision_Left &&
    Collision_Right == Old_Collision_Right &&
    Collision_Back == Old_Collision_Back;
  if (Unchanged) return;

  Update_Collision_Display(Collision_Forward, Collision_Left, Collision_Right, Collision_Back);
  if (Collision_Forward) Serial.println("Collision: Forward");
  if (Collision_Left) Serial.println("Collision: Left");
  if (Collision_Right) Serial.println("Collision: Right");
  if (Collision_Back) Serial.println("Collision: Back");

  Old_Collision_Forward = Collision_Forward;
  Old_Collision_Left = Collision_Left;
  Old_Collision_Right = Collision_Right;
  Old_Collision_Back = Collision_Back;
}
