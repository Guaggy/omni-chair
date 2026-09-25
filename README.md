# Omni Chair firmware

Firmware for our mecanum-wheel wheelchair, running on a LilyGO T-Display S3.
A USB joystick drives the four wheels through two Sabertooth controllers,
and five Sharp PSD sensors plus an LD06 LiDAR slow the chair down or stop it
near obstacles. You can follow what's going on on the built-in screen or on
a small web page over WiFi.

## Hardware

- LilyGO T-Display S3 (ESP32-S3)
- USB Host Shield with the joystick
- 2x Sabertooth, front is address 128 and back is 129
- LD06 LiDAR looking forward
- 5x Sharp GP2Y0A21 PSDs (front left, front right, both sides and back)

All pins are in [src/configs/pins.h](src/configs/pins.h).

## Settings

Everything you might want to tweak is in
[src/configs/config.h](src/configs/config.h). The toggles you change with the
buttons or the web page are saved and come back after a reboot, except
collision avoidance which always starts on.

| Button | Does |
|---|---|
| 1-5 | Screens: driving, PSD, LiDAR, config, stats |
| 7 | Collision avoidance on/off |
| 8 | Hardstop (stop instead of crawl) |
| 9 | Squared input for finer control |
| 10 | Ignore front PSDs, the LiDAR guards the front alone |
| 11 | WiFi on/off |
| 12 | Start/stop recording |

## Collision avoidance

The chair only slows down in the direction it's moving, so driving
diagonally into a wall makes it slide along the wall instead of turning.
The PSDs slow it to 30 % within 50 cm and 10 % within 30 cm. The LiDAR slows
the front to 70 % within 80 cm.

With the front PSDs ignored (for when your legs are in the way), the LiDAR
does the front on its own and stops the chair at 40 cm. It stays stopped until
the obstacle is more than 50 cm away or you reverse. Turning is slowed when
something is close but never fully blocked, so you can always turn away.

## Web page and recording

Connect to the open WiFi **OmniChair** and go to `http://omnichair.local` (or
`192.168.4.1`). There you get the LiDAR radar, PSD bars, the joystick, stats, a
log and the settings buttons. There's no password, so anyone nearby can
change things.

Recording saves a row every 100 ms (up to 20 minutes) and you download it as CSV
from the web page. It's gone after a reboot. Downloading stops the motors,
so centre the joystick afterwards to drive again.