#!/usr/bin/env python3

from serial.tools import list_ports

device_list = list_ports.comports()
for device in device_list:
    if device.manufacturer == "Xsens" or device.product == "Motion Tracker Dev. Board":
        print("Device Serial Port: ",device.device, " Device Serial Number: ", device.serial_number)