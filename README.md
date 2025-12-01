# Embedded Systems Python Integration Project

## Objective
The main objective of this project is to integrate Python programming with embedded systems to enable control and interaction with hardware devices such as an Arduino.

## Success Criteria
To consider this project successful, the following goals must be met:

- **PyQt GUI** to read in and display the current temperature readings from the Arduino  
- **Active control** from the GUI to the Arduino for fan operation  
- **Automated fan toggling** based on sensor input and predefined temperature thresholds  
- **Data storage and visualization** using MatPlotLib to show temperature trends over time

## Required Packages
- **PyQt** – for building the graphical user interface  
- **PySerial** – for communication between the Arduino and Python  
- **MatPlotLib** – for creating graphs of temperature data

## Planned Functionality (From Project Proposal)
- **Bang-bang control:** Turn the fan on or off based on temperature thresholds  
- **Variable control:** Dynamically adjust fan behavior to maintain a target temperature  
- **Emergency response mode:** Special high-temperature alert or response, e.g., triggered with a lighter during testing

## Use Instructions with Arduino
- Assemble arduino circuit
- Connect to laptop/desktop
- Run ControlwithGUI program and test with editing setpoints
- Use GUI to edit PID control variables

## Use Instructions without Arduino
- Run ControlwithGUI file program
- Simulator is included in this code
- Edit PID variables
- Run program and look at chart and edit varibales as needed