import PyQt5
import matplotlib.pyplot as plt
import numpy
import serial
import time

ser = serial.Serial('COM8', 9600, timeout=1)
time.sleep(2)

if not ser.is_open:
    print("Serial port not open")
    exit()

SETPOINT = 26

def read_temp():
    try:
        line = ser.readline().decode().strip()
        temp_str, pwm_str = line.split(",")
        return float(temp_str)
    except:
        return None

def fan_on():
    ser.write(bytes([255]))  # full speed

def fan_off():
    ser.write(bytes([0]))    # off


while True:
    temp = read_temp()

    if temp is not None:
        print(f"Temp: {temp:.2f} °C")

        if temp >= SETPOINT:
            fan_on()
            print("Fan: ON (255)")
        else:
            fan_off()
            print("Fan: OFF (0)")

    time.sleep(0.5)
