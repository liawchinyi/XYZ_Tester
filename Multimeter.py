import serial
import time

def read_voltage_xdm1000(serial_object):
    """
    Reads the DC voltage value from an XDM1000 series multimeter using SCPI commands.
    Assumes the serial port object for the multimeter is already open and configured.

    Args:
        serial_object (serial.Serial): The already open pyserial Serial object for the multimeter.

    Returns:
        float or None: The DC voltage value in Volts, or None if an error occurs.
    """
    if serial_object is None or not serial_object.is_open:
        print("Error: Multimeter serial port is not open. Cannot read voltage.")
        return None

    try:
        # Clear any existing buffer before sending commands
        serial_object.reset_input_buffer()
        serial_object.reset_output_buffer()

        # Set the multimeter to measure DC Voltage (SCPI command)
        serial_object.write(b"FUNCtion 'VOLTage:DC'\n")
        time.sleep(0.1) # Short delay for command processing

        # Request the voltage measurement
        serial_object.write(b"MEASure:VOLTage:DC?\n")
        
        # Read the response
        response = serial_object.readline().decode('utf-8').strip()
        
        # Parse the response
        if response:
            try:
                voltage = float(response)
                return voltage
            except ValueError:
                print(f"Error: Could not convert multimeter response to float: '{response}'")
                return None
        else:
            print("Error: No response received from multimeter within timeout.")
            return None

    except serial.SerialException as e:
        print(f"Multimeter Serial Port Error: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during multimeter reading: {e}")
        return None

if __name__ == "__main__":
    # --- Configuration ---
    # IMPORTANT: Replace 'COM13' with the actual serial port for your multimeter.
    multimeter_port = 'COM13'   
    # IMPORTANT: Adjust the baud rate to match your multimeter's settings.
    multimeter_baudrate = 115200 
    # -------------------

    multimeter_ser = None # Initialize to None

    try:
        # Open the serial port for the multimeter
        multimeter_ser = serial.Serial(
            port=multimeter_port,
            baudrate=multimeter_baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=5 # Set a timeout for reading
        )
        print(f"Connected to multimeter on {multimeter_port} at {multimeter_baudrate} baud.")
        time.sleep(1) # Give multimeter time to initialize

        # Query multimeter's identification (good practice to confirm connection)
        multimeter_ser.write(b"*IDN?\n")
        idn_response = multimeter_ser.readline().decode('utf-8').strip()
        print(f"Multimeter Identification: {idn_response}")
        multimeter_ser.reset_input_buffer() # Clear buffer after IDN
        multimeter_ser.reset_output_buffer()

        print("\nAttempting to read DC Voltage from multimeter...")
        voltage_value = read_voltage_xdm1000(multimeter_ser)

        if voltage_value is not None:
            print(f"\nSuccessfully Read Voltage: {voltage_value:.4f} Volts")
        else:
            print("\nFailed to read DC Voltage value.")

    except serial.SerialException as e:
        print(f"Connection Error: {e}")
        print("Please check:\n- Correct **COM port** and **baud rate**.\n- Multimeter is **connected and powered on**.\n- No other software is using this serial port (e.g., multimeter software).")
    except Exception as e:
        print(f"An unexpected error occurred during script execution: {e}")
    finally:
        if multimeter_ser is not None and multimeter_ser.is_open:
            multimeter_ser.close()
            print("Multimeter serial port closed.")