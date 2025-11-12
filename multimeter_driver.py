# multimeter_driver.py

import pyvisa

def read_3458a_resistance(visa_address):
    """
    Connects to an HP 3458A multimeter, measures resistance, and prints the result.
    This version incorporates user-provided settings for timeout, chunk size, and delay,
    and uses a 'with' statement for reliable resource management.

    Args:
        visa_address (str): The VISA address of the instrument.
    """
    try:
        # Use a 'with' statement for automatic resource closing and cleanup
        rm = pyvisa.ResourceManager()
        with rm.open_resource(visa_address) as instrument:
            
            # --- User-provided communication settings ---
            # Set a longer timeout. Timeout is in milliseconds.
            instrument.timeout = 1000
            
            # Set the communication chunk size.
            instrument.chunk_size = 50
            
            # Set the inter-command delay in seconds.
            instrument.delay = 0.05 # Using a small delay to be safe
            
            # Set the termination character for both writing and reading.
            instrument.write_termination = '\n'
            instrument.read_termination = '\n'
                       
            # Configure the multimeter for resistance measurement
            # We will use 'FRES' for 4-wire ohms measurement
            instrument.write('OHMF')
            # You can also set the range explicitly, e.g., for 1k Ohm range
            # instrument.write('FRES 1E3')

            # The 'READ?' command takes a new measurement and returns the result.
            resistance_value = instrument.query('READ?')
            resistance = float(resistance_value)
            
            print(f"Measured Resistance: {resistance} Ohm")
            return resistance

    except pyvisa.errors.VisaIOError as e:
        print(f"Error communicating with the instrument: {e}")
        print("Please ensure the instrument is connected, powered on, and the VISA address is correct.")
        
def send_gcode(serial_object, command, read_response=True):

    if serial_object is None or not serial_object.is_open:
        print("Error: Printer serial port is not open. Cannot send command.")
        return None # Return None on error or unopen port

    try:
        command_bytes = (command + '\n').encode('utf-8')
        print(f"Sending printer command: {command}")
        serial_object.write(command_bytes)

        full_response = b''
        if read_response:
            while True:
                line = serial_object.readline()
                if not line:
                    break
                full_response += line
                if b'ok' in line.lower() or b'error' in line.lower() or b'resend' in line.lower():
                    break
            
            decoded_full_response = full_response.decode('utf-8').strip()
            if decoded_full_response:
                print("Printer response:")
                print(decoded_full_response)
                return decoded_full_response
            else:
                print("No response from printer within timeout.")
                return None
        return None # Return None if read_response is False

    except serial.SerialException as e:
        print(f"Error: Serial communication error with printer ({serial_object.port}). {e}")
        print("Please check if the port is still connected or not in use by another program.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred while sending '{command}' to printer: {e}")
        return None
    