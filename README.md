# The Lunar Lander
This project is a Verilog-based implementation of a Lunar Lander simulation, which models the dynamics of a lunar landing sequence. The system simulates a lander's descent, 
taking into account factors such as altitude, velocity, fuel consumption, and thrust, all while displaying the current status on seven-segment displays and LED indicators.

Key Features:

Lunar Lander Module: Simulates the lander's altitude, velocity, fuel, and thrust based on input commands, gravity, and current fuel levels. The module 
continuously updates the lander's state as it descends.

Display Interface: Outputs critical information such as altitude, velocity, fuel, and thrust on seven-segment displays, while also indicating landing or 
crashing events through LED indicators.

Control Unit: Manages the control signals that determine whether the lander has successfully landed or crashed based on the velocity and altitude.

Arithmetic Unit: Performs the necessary calculations to update the lander's altitude, velocity, and fuel levels in real-time.

Memory Unit: Stores and updates the current state of the lander, including altitude, velocity, fuel, and thrust, on every clock cycle.

Input Handling: Processes user inputs for controlling the thrust, allowing the simulation of different landing scenarios.

This project is designed to provide a detailed and realistic simulation of the challenges involved in landing a spacecraft on the lunar surface, making 
it a valuable tool for understanding control systems and digital logic design.
