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

## How to Use Arduino Code
- Download the Arduino code in the ArduinoFiles folder
- Install and open the Arduino IDE
- Connect your Arduino to the computer, select the correct hardware, and port
- Upload the provided file to the Arduino  


