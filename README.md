# Numkey
Functional external numerical keypad using only one keyboard switch on a H-bot kinematic.

## Features
- calibration process
- fully functional numlock
- H-bot kinematic
- sacrificial component

## How it works
An ESP32 S3 captures a photo of where your finger is, and then sends it to connected PC via USB C. Then, local neural network searches for your finger, transforms it location on the photo to location above the device and sends the keyboard switch to a location of corresponding numpad key and updates the screen. When the ESP32 detect that the switch is pressed it sends that information to PC and a python script emulates keyboard inputs.

## Purpose
Main purpose of this project was for me to learn more about neural networks, controlling stepper motors and learning about different kinematics.

## Requirements
- Windows PC
- python 3
- arduino IDE (or other software to upload firmware to ESP32)

## How to make
-  How to print:
  1. [`chassis.stl`](./models/chassis.stl) - ensure that chosen filament can withstand stepper motors temp, use low layer height/variable layer height, use organic supports.
  2. [`carriages.stl`](./models/carriages.stl) - use low layer height/variable layer height.
  3. [`sacrificial.stl`](./models/sacrificial.stl) - exact settings may need tuning for different printers, don't use more then 2 wall loops, use low infill (<=5%).
  4. [`rest.stl`](./models/rest.stl) - no supports.
-  How to assemble:
  1. Assemble the parts accordingly to [`assembly.mp4`](./assets/assembly.mp4).
  2. Upload [`ESP32_firmware.ino`](./code/ESP32_firmware.ino) to ESP32-S3.
  3. Install all the wires accordingly to [`wiring.jpg`](./assets/wiring.jpg).
-  How to use:
  1. Download all the files from [`code/`](./code/).
  2. Plug the ESP32 into the PC via USB.
  3. Run [`main_code.py`](./code/main_code.py)
