import serial
import time

def read_resistance_xdm1000(port='COM13', baudrate=115200, timeout=5):
    """
    Reads the resistance value from an XDM1000 series multimeter using SCPI commands.

    Args:
        port (str): The serial port the multimeter is connected to (e.g., 'COM13' on Windows, '/dev/ttyUSB0' on Linux).
        baudrate (int): The baud rate for serial communication. Common values include 9600, 19200, 38400, 57600, 115200.
                        Check your multimeter's manual for the correct baud rate.
        timeout (int): The read/write timeout in seconds.

    Returns:
        float or None: The resistance value in Ohms, or None if an error occurs.
    """
    try:
        # Open the serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        print(f"Connected to {port} at {baudrate} baud.")
        time.sleep(1) # Give some time for the connection to establish

        # Clear any existing buffer
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 1. Query the multimeter's identification to ensure communication is working
        ser.write(b"*IDN?\n")
        idn_response = ser.readline().decode('utf-8').strip()
        print(f"Multimeter Identification: {idn_response}")

        # 2. Set the multimeter to measure resistance (optional, but good practice)
        ser.write(b"FUNCtion 'RESistance'\n")
        time.sleep(0.1) # Short delay for command processing

        # 3. Request the resistance measurement
        ser.write(b"MEASure:RESistance?\n")
        
        # 4. Read the response
        response = ser.readline().decode('utf-8').strip()
        
        # 5. Parse the response
        if response:
            try:
                resistance = float(response)
                print(f"Measured Resistance: {resistance} Ohms")
                return resistance
            except ValueError:
                print(f"Error: Could not convert response to float: '{response}'")
                return None
        else:
            print("Error: No response received from multimeter.")
            return None

    except serial.SerialException as e:
        print(f"Serial Port Error: {e}")
        print("Please check if the correct port is selected and if the multimeter is connected and powered on.")
        print("Also, ensure no other application is using the serial port.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    # --- Configuration ---
    # IMPORTANT: Replace 'COM13' with the actual serial port for your multimeter.
    # On Linux, this might be something like '/dev/ttyUSB0' or '/dev/ttyACM0'.
    # IMPORTANT: Adjust the baud rate to match your multimeter's settings.
    # Common baud rates are 9600, 19200, 38400, 57600, 115200.
    multimeter_port = 'COM13' 
    multimeter_baudrate = 115200 # Updated baud rate to 115200
    # -------------------

    print(f"Attempting to read resistance from multimeter on {multimeter_port}...")
    resistance_value = read_resistance_xdm1000(multimeter_port, multimeter_baudrate)

    if resistance_value is not None:
        print(f"\nFinal Result: {resistance_value} Ohms")
    else:
        print("\nFailed to read resistance value.")