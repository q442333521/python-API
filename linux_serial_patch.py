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
        ser = None
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
            print("Could not find any available serial ports.")
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            if ports:
                print("Available ports:")
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


