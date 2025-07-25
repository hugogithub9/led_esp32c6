Project Overview
This is a complete and time-intensive project exploring how to control LEDs (including RGB LEDs) on the ESP32-C6 using Rust. It includes several examples:

led1: A basic LED control example : turns an LED on/off, even though the actual board uses an RGB LED.


led3: Builds on an adapted implementation using an RGB struct and a no_pixel function to change the LED color. The LED also blinks inside the main function.


led4: First attempt at integrating DEVS formalism into the project.

Implements a basic Generator atomic model.

The generator randomly outputs either true or false.

Based on this value, a color is selected and applied to the RGB LED.

The output is directly managed in the simulate() function using the output argument.


led5: Introduces three atomic models:

Button: Simulates a button press by generating a random press duration between 0 and 10 seconds.

Controller: Calculates how long the button was pressed and outputs a color command ('g', 'r', or 'b') based on the duration.

Activator: Receives the color command and lights up the RGB LED using neopixel.


led6: Same logic as led5, but the Activator block is removed.

The coupled model is updated accordingly.

Now, the color command output from the Controller is handled directly inside the simulate() function, which activates the LED based on the value.


led7: Similar to led6, but now a real physical button is connected to GPIO14 on the ESP32-C6 board.

The Button atomic model is modified to read the actual pin state using the function is_high().

Random timing is removed and the input is now based on actual hardware interaction.


led8: Streamlines the model:

The Button component is replaced by direct external input.

A new input_handler() function is introduced to simulate button behavior, generating DEVS inputs manually.

The DEVS model now focuses only on the Controller, while inputs are injected from outside.



This project offers a clear, incremental path to understanding:

Embedded LED control on the ESP32-C6,

Real-time input/output integration,

DEVS formalism with atomic and coupled models,

How to progressively add inputs and outputs to a DEVS model.
By moving from simulation to real hardware interaction, and from static logic to modular, event-driven design, this project demonstrates how DEVS can be applied to real embedded systems in Rust.
