Project Overview
This is a complete and time-intensive project exploring how to control LEDs (including RGB LEDs) on the ESP32-C6 using Rust. It includes several examples:

led1: A basic LED control example — turns an LED on/off, even though the actual board uses an RGB LED.

led3: Builds on an adapted implementation using an RGB struct and a no_pixel function to change the LED color. The LED also blinks inside the main function.

led4: First attempt at integrating DEVS formalism into the project.
Implements a basic Generator atomic model.
The generator randomly outputs either true or false.
Based on this value, a color is selected and applied to the RGB LED.
The output is directly managed in the simulate() function using the output argument.
