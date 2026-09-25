#pragma once

// Pins for the LilyGO T-Display S3, the screen pins are in platformio.ini

const int Peripheral_Power_Enable_Pin = 15;

// Motor serial to the Sabertooths
const int Motor_RX_Pin = 43;
const int Motor_TX_Pin = 44;

// USB Host Shield, if you change these also update the "Omni Chair" lines in lib/USB_Host_Shield_2.0
const int USB_SS_Pin = 18;
const int USB_Interrupt_Pin = 17;
const int USB_Clock_Pin = 13;
const int USB_MISO_Pin = 10;
const int USB_MOSI_Pin = 11;

// LiDAR only sends, so it needs just one pin
const int Lidar_RX_Pin = 21;

// PSD sensors, don't use GPIO17 or 18 since the board pulls them up to 3.3 V
const int Sensor_Front_Left_Pin = 1;
const int Sensor_Front_Right_Pin = 2;
const int Sensor_Side_Left_Pin = 16;
const int Sensor_Side_Right_Pin = 12;
const int Sensor_Back_Pin = 3;
