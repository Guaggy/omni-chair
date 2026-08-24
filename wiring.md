# Wiring Diagram
<img src="images/sample_wiring.jpg" alt="Sample Witing" width="500">

## Pinout References and Images
The following are pinouts of both the ttgo and the USB Host Shield v1.1. 

**NOTE**: The USB Host Shield image attached is not the exact part but it is a clone so pinout is mostly the same.

### ttgo
<img src="images/ttgo.jpg" alt="ttgo pinout" width="500">

### USB Host Shield
<img src="images/usb_shield_pinout.jpg" alt="USB Host Shield" width="500">

## Pin Wiring
Both the USB Host Shield, Sabertooth motors and the PSD sensors are wired to the ESP32 through the following GPIO pins:

### USB HOST Shield  
```
ttgo   : USB Host Shield
------------------------
5v     : 5v
GND    : GND
GPIO33 : 10
GPIO17 : 4 (Digital, not Analog)
GPIO25 : 13
GPIO27 : 12
GPIO26 : 11
```

### Sabertooth Motors
```
ttgo   : Sabertooth
-------------------
GPIO21 : S2 (RX) (Red Wire) 
GPIO22 : S1 (TX) (Green Wire)
```

### PSD Sensors
All sensors are wired to both 5v and GND. The Data wire for each sensor goes to each corresponding analog GPIO pin:
```
PSD Sensor Wires:
Red Wire = 5v
Black Wire = GND
Yellow Wire = Data

ttgo   : PSD Sensors
--------------------
GPIO12 : FRONT_LEFT
GPIO13 : FRONT_RIGHT
GPIO15 : SIDE_LEFT
GPIO2  : SIDE_RIGHT
GPIO39 : BACK
```

### Electric Diagram
<img src="images/wiring_diagram.jpg" alt="Wiring Diagram" width="1000">

The Fritzing Diagram can be found in the [Fritzing](fritzing) folder.
