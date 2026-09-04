# numkey
Functional external numerical keypad using only one keyboard switch on a H-bot kinematic.

## Features
- calibration process
- fully functional numlock
- H-bot kinematic
- sacrificial component

# Used parts
- NEMA17 stepper motors
- 70cm timing belt

## How it works
An ESP32 S3 captures a photo of where your finger is, and then sends it to connected PC via USB C. Then, local neural network searches for your finger, transforms it location on the photo to location above the device and sends the keyboard switch to a location of corresponding numpad key and updates the screen. When the ESP32 detect that the switch is pressed it sends that information to PC and a python script emulates keyboard inputs.

## Purpose
Main purpose of this project was for me to learn more about neural networks, controlling stepper motors and learning about different kinematics.
