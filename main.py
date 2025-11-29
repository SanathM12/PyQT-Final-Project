import PyQt5
import serial
import matplotlib.pyplot as plt
import numpy

ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)

if not ser.is_open:
    print("Serial port not open")
    exit()

SETPOINT = 30.0

def read_temp():
    try:
        line = ser.readline().decode().strip()
        return float(line)
    except:
        return None

def fan_on():
    ser.write(b'ON\n')

def fan_off():
    ser.write(b'OFF\n')


while True:
    temp = read_temp()

    if temp is not None:
        print(f"Temp: {temp:.2f} °C")

        if temp >= SETPOINT:
            fan_on()
            print("Fan: ON")
        else:
            fan_off()
            print("Fan: OFF")

    time.sleep(0.5)