# ODW_ROBOT_USBJOYSTICK_V2

Reimplementation of the joystick connectivity for the Omni-Directional Wheelchair.

## V2 Differences
- Use USB Host Shield 2.0 Library instead of UART Serial
- PSD Sensor Collision System
## Requirements
If you are using the ttgo T-Display v1.1, you must use this USB Host Shield 2.0 Library: https://github.com/Ay1tsMe/USB_Host_Shield_2.0

Otherwise you can use the main repo if you are using generic ESP32 devices or Arduino Uno's

## Pin Specifications
### SPI
The USB Host Shield is wired up to the ttgo through the SPI pins on both boards. The following GPIO pins are the SPI pins for the ttgo:
```
GPIO33 : SS
GPIO17 : INT
GPIO25 : SCK
GPIO27 : MISO
GPIO26 : MOSI
```

### Motors
The Wheelchair motors are wired up to the ttgo through the following GPIO pins:
```
// RX = S2
// TX = S1
#define RX_PIN 21
#define TX_PIN 22
```
### PSD Collision System
There are 5 sensors that are wired up to the t-display v1.1 that operate the collision system. These pins include:
```
// PSD Sensors
#define FRONT_LEFT 12
#define FRONT_RIGHT 13
#define SIDE_LEFT 15
#define SIDE_RIGHT 2
#define BACK 39
```

Each sensor has a distance variable that is measured in cm. If the user would like to adjust the distance that the collision system activates, then you can change the cm value of each distance variable:

```
// Change from 50cm to 60cm
// From:
if (side_left_distance < 50) {

// To:
if (side_left_distance < 60) {
```
If you want to disable the collision system entirely, there is a global variable called collisionEnabled that can be changed from true to false.

```
// Collision Enabled
bool collisionEnabled = true;

// Collision Disable
bool collisionEnabled = false;
```
### Wiring Diagram
For more detailed wiring diagram, [click here](wiring.md)

