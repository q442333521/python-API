#!/usr/bin/env python3
"""
Fixed version of headless_commander.py with Linux serial port support
"""
import sys
import os
import platform

# Add serial port initialization for Linux
def initialize_serial():
    import serial
    import serial.tools.list_ports
    
    my_os = platform.system()
    ser = None
    
    if my_os == "Windows":
        # Try to read the COM port from a file
        try:
            with open("com_port.txt", "r") as f:
                com_port_str = f.read().strip()
                ser = serial.Serial(port=com_port_str, baudrate=3000000, timeout=0)
                print(f"Connected to saved COM port: {com_port_str}")
        except (FileNotFoundError, serial.SerialException):
            # If the file doesn't exist or the port is invalid, ask the user
            while True:
                try:
                    com_port = input("Enter the COM port (e.g., COM9): ")
                    ser = serial.Serial(port=com_port, baudrate=3000000, timeout=0)
                    print(f"Successfully connected to {com_port}")
                    # Save the successful port to the file
                    with open("com_port.txt", "w") as f:
                        f.write(com_port)
                    break
                except serial.SerialException:
                    print(f"Could not open port {com_port}. Please try again.")
    else:
        # Linux/Mac system
        # Try to read the serial port from a file
        try:
            with open("com_port.txt", "r") as f:
                com_port_str = f.read().strip()
                ser = serial.Serial(port=com_port_str, baudrate=3000000, timeout=0)
                print(f"Connected to saved serial port: {com_port_str}")
        except (FileNotFoundError, serial.SerialException):
            # If the file doesn't exist or the port is invalid, try default ports
            default_ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
            for port in default_ports:
                try:
                    ser = serial.Serial(port=port, baudrate=3000000, timeout=0)
                    print(f"Successfully connected to {port}")
                    # Save the successful port to the file
                    with open("com_port.txt", "w") as f:
                        f.write(port)
                    break
                except serial.SerialException:
                    continue
            
            if ser is None:
                print("Could not find any available serial ports. Available ports:")
                ports = serial.tools.list_ports.comports()
                for port in ports:
                    print(f"  {port.device}")
                
                while True:
                    try:
                        com_port = input("Enter the serial port (e.g., /dev/ttyACM0): ")
                        ser = serial.Serial(port=com_port, baudrate=3000000, timeout=0)
                        print(f"Successfully connected to {com_port}")
                        # Save the successful port to the file
                        with open("com_port.txt", "w") as f:
                            f.write(com_port)
                        break
                    except serial.SerialException:
                        print(f"Could not open port {com_port}. Please try again.")
    
    return ser

# Now run the original script with our serial initialization
if __name__ == "__main__":
    import subprocess
    
    # First initialize the serial port
    ser = initialize_serial()
    if ser:
        print(f"Serial port successfully initialized: {ser.port}")
        ser.close()  # Close it, let the main script handle it
    
    # Now we need to patch the original script
    print("Patching original script...")

