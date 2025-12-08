# Embedded Systems Python Integration Project

## Objective
The main objective of this project is to integrate Python programming with embedded systems (e.g. Arduinos) to enable control and interaction with hardware devices (e.g. a fan).

## Success Criteria
To consider this project successful, the following goals must be met:

- **PyQt GUI** to read in and display the current temperature readings from the Arduino  
- **Active control** from the GUI to the Arduino for fan operation  
- **Automated fan toggling** based on sensor input and predefined temperature thresholds  
- **Data storage and visualization** using MatPlotLib to show temperature trends over time

## Required Packages
To run this application, you must have the following Python packages installed

- **PyQt** – for building the graphical user interface  
- **PySerial** – for communication between the Arduino and Python  
- **MatPlotLib** – for creating graphs of temperature data

## I. How to Use Arduino Code
- Download the Arduino code in the ArduinoFiles folder
- Install and open the Arduino IDE
- Connect your Arduino to the computer, select the correct hardware, and port
- Wire up the Arduino to the temperature sensor and fan using the wiring diagram below
- Upload the provided file to the Arduino

## II. How to wire the Arduino
- The DHT11 data pin goes to digital 3 on the Arduino
- The DHT11 positive pin goes to the 5V power on the Arduino
- The DHT11 negative pin goes to the ground pin on the Arduino
- The H-Bridge ENA pin goes to digital 6 on the Arduino
- The H-Bridge IN1 pin goes to digital 5 on the Arduino
- The H-Bridge IN2 pin goes to digital 5 on the Arduino
- The 12V positive and negative leads on the H-Bridge go to the positive and negative leads on your external power supply. I utilised a 9V battery, and it worked just fine.
- Connect the 5V power pin on the H-Bridge to the 5V output on the Arduino
- The motor leads connect to the screw terminals on the H-Bridge. Switching these cables reverses the forward direction of the motor.
- Connect a Mini USB from the Arduino Nano to your PC's USB port for the required connection.

<img width="699" height="403" alt="image" src="https://github.com/user-attachments/assets/047a4f02-1a23-4c38-9e53-0b966e7ce07b" />

## III. How to use PyQT GUI
The GUI is designed for monitoring and switching between manual and automatic control modes.

**Initial Setup** 
- Verify that the PORT is the same as the Arduino IDE selected port
- Run the Python Program (ControlwithGUI.py)
- The application should be connected to the specified PORT

**Monitoring System Status** 
- Mode: Shows current operational mode (Manual, Bang-Bang, PID, Emergency)
- Current Temp: Shows a live temperature reading from the sensor (or simulation)
- Target Temp: The desired temperature setpoint
- Fan Output: The PWM value currently being sent to the fan

**Setting Target Temperature/Other Parameters** 
- Ensure that the Control Loop is off before changing parameters
- Change all necessary parameters as needed
- Click "Apply Params" to apply new parameters

<img width="715" height="398" alt="image" src="https://github.com/user-attachments/assets/ed4df266-8c8b-4d89-aef3-0827a31135fc" />






