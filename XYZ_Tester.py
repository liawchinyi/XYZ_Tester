'''
Name: Chin-Yi LIAW
Company: National Metrology Centre
Date: 17 June 2025
'''

import serial
import time
import re # Import regex module for robust stripping
import csv
import math # For math.isclose for float comparisons
import sys  # Import the sys module for sys.exit()
import matplotlib.pyplot as plt # Import for plotting
import numpy as np # Import for numerical operations, if needed by matplotlib

# --- Global Serial Port Objects ---
# These will be initialized in the __main__ block
printer_ser = None
multimeter_ser = None 

# --- Global State for Extruder Position Tracking ---
# This dictionary will store the last E-value commanded for each (X, Y) knob coordinate.
# It persists across different target_total_r iterations.
last_e_positions = {} 

# --- Global State for Printer's Last Known XY Position ---
# This will track the actual (X,Y) coordinates the printer physically moved to last.
# This helps optimize Z lifts/lowers for consecutive operations on the same knob.
current_printer_xy_pos = (None, None) 

def read_resistance_xdm1000(serial_object):
    """
    Reads the DC resistance value from an XDM1000 series multimeter using SCPI commands.
    Assumes the serial port object for the multimeter is already open and configured.

    Args:
        serial_object (serial.Serial): The already open pyserial Serial object for the multimeter.

    Returns:
        float or None: The DC resistance value in Ohms, or None if an error occurs.
    """
    if serial_object is None or not serial_object.is_open:
        print("Error: Multimeter serial port is not open. Cannot read resistance.")
        return None

    try:
        # Clear any existing buffer before sending commands
        serial_object.reset_input_buffer()
        serial_object.reset_output_buffer()

        # 1. Set the multimeter to measure resistance
        print("  Sending: FUNCtion 'RESistance'") 
        serial_object.write(b"FUNCtion 'RESistance'\n")
        time.sleep(0.1) # Short delay for command processing

        # 2. Request the resistance measurement
        print("  Sending: MEASure:RESistance?") 
        serial_object.write(b"MEASure:RESistance?\n")
        
        # 3. Read the response
        response = serial_object.readline() # Read bytes directly
        # Attempt decoding with 'latin-1' as it successfully resolved previous errors
        decoded_response = response.decode('latin-1').strip()
        print(f"  Raw Multimeter Response (decoded with latin-1): '{decoded_response}'") # Print raw response for debugging
        
        # Parse the response
        if decoded_response:
            # This regex matches any non-digit, non-decimal point, non-sign character at the end of the string
            # and removes it. This handles units like 'VDC', 'V', 'mV', 'A', 'ohm', '¦¸', etc.
            numeric_response = re.sub(r'[^\d.+\-eE]*$', '', decoded_response) 
            # Remove any leading/trailing whitespace again after stripping
            numeric_response = numeric_response.strip()

            try:
                resistance = float(numeric_response)
                return resistance
            except ValueError:
                print(f"Error: Could not convert multimeter response to float: '{decoded_response}' (after stripping: '{numeric_response}')")
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

def send_gcode(serial_object, command, read_response=True):
    """
    Sends a G-code command using the provided serial port object.

    Args:
        serial_object (serial.Serial): The already open pyserial Serial object for the printer.
        command (str): The G-code command to send (e.g., 'G1 X100 F1000').
        read_response (bool): Whether to read and print the printer's response.
    """
    if serial_object is None or not serial_object.is_open:
        print("Error: Printer serial port is not open. Cannot send command.")
        return

    try:
        # Ensure the command ends with a newline character,
        # which is standard for Marlin G-code parsing.
        command_bytes = (command + '\n').encode('utf-8')

        print(f"Sending printer command: {command}")
        serial_object.write(command_bytes)

        if read_response:
            response = b''
            # Marlin typically sends 'ok' or 'Error' as the final acknowledgement.
            while True:
                line = serial_object.readline()
                if not line: # Timeout occurred or no more data
                    break
                response += line
                if b'ok' in line.lower() or b'error' in line.lower() or b'resend' in line.lower():
                    break

            if response:
                print("Printer response:")
                print(response.decode('utf-8').strip())
            else:
                print("No response from printer within timeout.")

    except serial.SerialException as e:
        print(f"Error: Serial communication error with printer ({serial_object.port}). {e}")
        print("Please check if the port is still connected or not in use by another program.")
    except Exception as e:
        print(f"An unexpected error occurred while sending '{command}' to printer: {e}")

def calculate_e_for_knob_resistance(knob_info, target_knob_r, extruder_e_values):
    """
    Calculates the closest E-value for a specific knob to achieve a target resistance contribution.
    Assumes a linear relationship between E0-E40 and the knob's expected min/max range.

    Args:
        knob_info (dict): Dictionary with knob details ('expected_min_r', 'expected_max_r').
        target_knob_r (float): The desired resistance contribution from this specific knob.
        extruder_e_values (list): List of possible E values (e.g., [0, 4, ..., 40]).

    Returns:
        float: The calculated E-value (or closest discrete E-value) for the knob.
    """
    e_start = float(extruder_e_values[0]) if extruder_e_values else 0.0
    e_end = float(extruder_e_values[-1]) if extruder_e_values else 0.0
    e_range_span = e_end - e_start
    r_knob_span = knob_info['expected_max_r'] - knob_info['expected_min_r']

    if r_knob_span == 0: # Handle fixed value knobs or misconfigured range (e.g., 0-0 Ohm)
        # If the knob has zero range and the target is also zero, return E0.
        # Otherwise, if it's asked to contribute something non-zero, it can't, so return E0.
        return e_start if math.isclose(target_knob_r, knob_info['expected_min_r'], rel_tol=1e-9) else e_start

    # Calculate ideal continuous E-value based on linear interpolation
    # Ensure target_knob_r is clamped within the knob's actual contribution range
    clamped_target_r = max(knob_info['expected_min_r'], min(knob_info['expected_max_r'], target_knob_r))

    if e_range_span > 0:
        ideal_e = e_start + (clamped_target_r - knob_info['expected_min_r']) * e_range_span / r_knob_span
    else:
        ideal_e = e_start # If E range is 0, only E0 is possible

    # Find the closest discrete E-value from the `extruder_e_values` list
    closest_e = extruder_e_values[0]
    min_diff = float('inf')
    for e_val in extruder_e_values:
        diff = abs(e_val - ideal_e)
        if diff < min_diff:
            min_diff = diff
            closest_e = e_val
    return closest_e

def set_all_decade_knobs(printer_ser, target_total_r, all_knob_definitions, extruder_e_values, extruder_feedrate, path_feedrate, last_e_positions_ref):
    """
    Calculates and applies the physical settings for the decade knobs on the General Radio 1433F
    to achieve the target_total_r. This involves moving to multiple XY locations and setting their E-axes.
    This version correctly decomposes the total resistance across relevant decades.

    Args:
        printer_ser (serial.Serial): The printer's serial object.
        target_total_r (float): The desired total resistance value to set.
        all_knob_definitions (list): List of dicts, defining all decade knobs.
        extruder_e_values (list): List of possible discrete E values (e.g., [0, 4, ..., 40]).
        extruder_feedrate (int): Feedrate for E movements.
        path_feedrate (int): Feedrate for XY movements.
        last_e_positions_ref (dict): Reference to the global last_e_positions dictionary to track E states.

    Returns:
        str: A string summarizing the knob settings applied, for CSV logging.
    """
    global current_printer_xy_pos # Declare intent to modify global variable

    print(f"  Calculating physical knob settings for target total resistance: {target_total_r:.2f} Ohms...")

    sorted_knobs_desc = sorted(all_knob_definitions, key=lambda p: p['decade_value'], reverse=True)

    # Dictionary to store the target E-value for each knob for this target_total_r
    # Initialize all to E0 (zero contribution)
    target_knob_e_settings = { (p['coords'][0], p['coords'][1]): 0.0 for p in all_knob_definitions }
    
    remaining_r_to_set = max(0.0, target_total_r)

    # Decomposition logic: Determine contribution for each knob
    for knob_info in sorted_knobs_desc:
        knob_coords = (knob_info['coords'][0], knob_info['coords'][1])
        knob_label = knob_info['label']
        decade_step_value = knob_info['decade_value']
        knob_min_r_contrib = knob_info['expected_min_r']
        knob_max_value = knob_info['expected_max_r']

        num_decade_steps = math.floor(remaining_r_to_set / decade_step_value)
        ideal_knob_r_contribution = num_decade_steps * decade_step_value
        knob_contribution_r = min(ideal_knob_r_contribution, knob_max_value)
        
        if knob_contribution_r < 1e-9:
            target_knob_e_settings[knob_coords] = 0.0
            print(f"    {knob_label} (step {decade_step_value} Ohm): Not contributing (too small). Setting to E0. Remaining: {remaining_r_to_set:.3f} Ohms")
            continue

        e_to_set = calculate_e_for_knob_resistance(knob_info, knob_contribution_r, extruder_e_values)
        
        e_range = extruder_e_values[-1] - extruder_e_values[0] # Should be 40
        actual_e_contrib_r = knob_min_r_contrib + \
                               (e_to_set - extruder_e_values[0]) * \
                               (knob_max_value - knob_min_r_contrib) / e_range \
                               if e_range > 0 else knob_min_r_contrib
        
        target_knob_e_settings[knob_coords] = e_to_set
        remaining_r_to_set -= actual_e_contrib_r
        remaining_r_to_set = max(0.0, remaining_r_to_set)

        print(f"    {knob_label} (step {decade_step_value} Ohm): Contributes {actual_e_contrib_r:.3f} Ohms (E={e_to_set:.2f}). Remaining: {remaining_r_to_set:.3f} Ohms")

    moves_to_execute = []
    for knob_coords, new_e_value in target_knob_e_settings.items():
        knob_label = next(p['label'] for p in all_knob_definitions if p['coords'] == knob_coords)
        old_e_value = last_e_positions_ref.get(knob_coords, 0.0)

        # Only add to moves if the E value needs to change
        if not math.isclose(old_e_value, new_e_value, rel_tol=1e-9):
             moves_to_execute.append((knob_coords, old_e_value, new_e_value, knob_label))
    
    # Sort moves by X then Y to minimize travel distance between knobs
    moves_to_execute.sort(key=lambda item: (item[0][0], item[0][1]))

    knob_settings_summary_list = []

    for knob_coords, old_e_value, new_e_value, knob_label in moves_to_execute:
        x_coord, y_coord = knob_coords

        # Check if the XY position is changing
        if (x_coord, y_coord) != current_printer_xy_pos:
            # 1. Retract Z to Z0 before moving XY
            print(f"  Lifting Z to Z0 before moving to {knob_label} (from {current_printer_xy_pos})...")
            send_gcode(printer_ser, "G1 Z0 F1000")
            send_gcode(printer_ser, "M400")
            time.sleep(0.2)

            # 2. Move to the new XY coordinates
            print(f"  Moving to {knob_label} (X:{x_coord}, Y:{y_coord})...")
            send_gcode(printer_ser, f"G1 X{x_coord} Y{y_coord} F{path_feedrate}")
            send_gcode(printer_ser, "M400")
            time.sleep(0.5)
            current_printer_xy_pos = (x_coord, y_coord) # Update global current position

            # 3. Set E to its last known position for THIS XY knob (before lowering Z)
            # This ensures the knob is "reset" or at its last state before interacting
            print(f"  Setting E to its previous state E:{old_e_value:.2f} for {knob_label}...")
            send_gcode(printer_ser, f"G1 E{old_e_value:.2f} F{extruder_feedrate}")
            send_gcode(printer_ser, "M400")
            time.sleep(0.2)

            # 4. Lower Z to measurement height
            print(f"  Lowering Z to Z1 for {knob_label}...")
            send_gcode(printer_ser, "G1 Z1 F1000") 
            send_gcode(printer_ser, "M400")
            time.sleep(0.2)
        else:
            # If XY is the same, just print that we're staying put
            print(f"  Staying at {knob_label} (X:{x_coord}, Y:{y_coord}) to adjust E-axis.")
            # If we are staying at the same knob but need to change E, ensure Z is at Z1
            # (it should already be from a previous action on this knob)
            send_gcode(printer_ser, "G1 Z1 F1000") # Ensure Z is at working height
            send_gcode(printer_ser, "M400")
            time.sleep(0.1)


        # 5. Set E to its new calculated value (happens regardless of XY move)
        print(f"  Setting E to its new state E:{new_e_value:.2f} for {knob_label}...")
        send_gcode(printer_ser, f"G1 E{new_e_value:.2f} F{extruder_feedrate}")
        send_gcode(printer_ser, "M400")
        time.sleep(0.5)

        last_e_positions_ref[knob_coords] = new_e_value
        knob_settings_summary_list.append(f"{knob_label}:E{new_e_value:.2f}")
    
    return ", ".join(knob_settings_summary_list)


# Printer Settings
PRINTER_PORT = 'COM18'      
PRINTER_BAUDRATE = 250000  

# Multimeter Settings
MULTIMETER_PORT = 'COM13'   
MULTIMETER_BAUDRATE = 115200 

# CSV Output File
OUTPUT_CSV_FILE = 'resistance_measurements.csv' 
PLOT_OUTPUT_FILE = 'resistance_plot.png' 

# Define XY Coordinates for each Decade Knob on the 1433F and their Expected Resistance Ranges
path_points = [
    {'coords': (2, 28),   'label': '0.001 Ohm Decade', 'decade_value': 0.001, 'expected_min_r': 0, 'expected_max_r': 0.01},
    {'coords': (15, 92),  'label': '0.01 Ohm Decade',  'decade_value': 0.01,  'expected_min_r': 0, 'expected_max_r': 0.1},
    {'coords': (75, 73),  'label': '1 Ohm Decade',     'decade_value': 1,     'expected_min_r': 0, 'expected_max_r': 10},
    {'coords': (90, 136), 'label': '10 Ohm Decade',    'decade_value': 10,    'expected_min_r': 0, 'expected_max_r': 100},
    {'coords': (149, 116),'label': '100 Ohm Decade',   'decade_value': 100,   'expected_min_r': 0, 'expected_max_r': 1000},
    {'coords': (164, 180),'label': '1K Ohm Decade',    'decade_value': 1000,  'expected_min_r': 0, 'expected_max_r': 10000},
    {'coords': (224, 160),'label': '10K Ohm Decade',   'decade_value': 10000, 'expected_min_r': 0, 'expected_max_r': 100000},
]

path_feedrate = 3000 
extruder_e_values = list(range(0, 41, 4)) 
extruder_feedrate = 500 

if __name__ == "__main__":
    measurement_data = [] 
    
    # Lists to store data for plotting (only successful measurements)
    plotted_resistances_commanded = [] 
    plotted_resistances_measured = []  

    min_desired_resistance = None
    max_desired_resistance = None
    desired_resistance_step = None

    while min_desired_resistance is None:
        try:
            min_input = input("Enter the minimum desired resistance value (Ohms) to set on the decade box: ")
            min_desired_resistance = float(min_input)
        except ValueError:
            print("Invalid input. Please enter a numerical value for minimum resistance.")
    
    while max_desired_resistance is None:
        try:
            max_input = input("Enter the maximum desired resistance value (Ohms) to set on the decade box: ")
            max_desired_resistance = float(max_input)
            if max_desired_resistance < min_desired_resistance:
                print("Maximum resistance cannot be less than minimum resistance. Please re-enter.")
                max_desired_resistance = None
        except ValueError:
            print("Invalid input. Please enter a numerical value for maximum resistance.")

    while desired_resistance_step is None:
        try:
            step_input = input("Enter the desired resistance step (e.g., 1 for 5, 6, 7; 0.1 for 5.1, 5.2): ")
            desired_resistance_step = float(step_input)
            if desired_resistance_step <= 0:
                print("Step must be positive. Please re-enter.")
                desired_resistance_step = None
        except ValueError:
            print("Invalid input. Please enter a numerical value for the step.")


    print(f"\nPlanning and generating paths to set total resistance values between {min_desired_resistance} Ohms and {max_desired_resistance} Ohms with a step of {desired_resistance_step} Ohms.")
    print("For each set resistance, a DC resistance reading will be taken from the multimeter.") 
    print(f"The commanded knob settings and measured resistances will be recorded to '{OUTPUT_CSV_FILE}'.") 
    print(f"A plot of Commanded Resistance vs. Measured Resistance will be saved to '{PLOT_OUTPUT_FILE}'.") 

    # --- Plotting setup (Interactive Mode) ---
    plt.ion() # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(10, 6))
    line, = ax.plot([], [], marker='o', linestyle='-', color='blue') # Create an empty line object
    ax.set_title('Commanded Resistance vs. Measured Resistance Sweep (Live)') 
    ax.set_xlabel('Commanded Resistance (Ohms)') 
    ax.set_ylabel('Measured Resistance (Ohms)') 
    ax.grid(True)
    
    # Set initial reasonable limits for resistance
    initial_x_min = min_desired_resistance * 0.9 if min_desired_resistance > 0 else -10 
    initial_x_max = max_desired_resistance * 1.1 if max_desired_resistance > 0 else 10 
    ax.set_xlim(initial_x_min, initial_x_max)
    ax.set_ylim(initial_x_min, initial_x_max) 

    try:
        printer_ser = serial.Serial(PRINTER_PORT, PRINTER_BAUDRATE, timeout=5)
        print(f"\nConnected to printer on {PRINTER_PORT} at {PRINTER_BAUDRATE} baud.")
        time.sleep(2)

        printer_ser.reset_input_buffer()
        printer_ser.reset_output_buffer()

        multimeter_ser = serial.Serial(
            port=MULTIMETER_PORT,
            baudrate=MULTIMETER_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=5
        )
        print(f"Connected to multimeter on {MULTIMETER_PORT} at {MULTIMETER_BAUDRATE} baud.")
        time.sleep(1)

        multimeter_ser.write(b"*IDN?\n")
        idn_response = multimeter_ser.readline().decode('utf-8').strip()
        print(f"Multimeter Identification: {idn_response}")
        multimeter_ser.reset_input_buffer()
        multimeter_ser.reset_output_buffer()

        send_gcode(printer_ser, "G90")
        send_gcode(printer_ser, "G28 Z")
        send_gcode(printer_ser, "G28 X")
        send_gcode(printer_ser, "G28 Y")
        send_gcode(printer_ser, "M17")
        send_gcode(printer_ser, "M400")
        send_gcode(printer_ser, "M114")
        
        # Initialize current_printer_xy_pos with the actual home position after G28
        current_printer_xy_pos = (0.0, 0.0) 

        print("\n--- Starting Targeted Resistance Setting & Resistance Measurement Sequence ---") 
        measurement_data.append(['Target_Resistance_Ohms', 'Measured_Resistance_Ohms', 'Knob_Settings_Applied']) 

        target_resistances_to_find = []
        current_target = min_desired_resistance
        while current_target <= max_desired_resistance + 1e-9: 
            target_resistances_to_find.append(current_target)
            current_target += desired_resistance_step
            
        if not target_resistances_to_find:
            print("No target resistance values generated within the given range and step. Exiting.")
            send_gcode(printer_ser, "M18")
            sys.exit(0)

        for target_r in target_resistances_to_find:
            print(f"\n--- Processing Target Resistance: {target_r:.2f} Ohms ---")
            
            knob_settings_applied_str = set_all_decade_knobs(
                printer_ser, 
                target_r, 
                path_points, 
                extruder_e_values, 
                extruder_feedrate, 
                path_feedrate, 
                last_e_positions
            )
            
            print(f"  Taking DC Resistance measurement...") 
            measured_r_data = read_resistance_xdm1000(multimeter_ser) 
            
            if measured_r_data is not None:
                print(f"  Target Resistance {target_r:.2f} Ohms: Measured Resistance {measured_r_data:.4f} Ohms. Settings: {knob_settings_applied_str}") 
                measurement_data.append([target_r, measured_r_data, knob_settings_applied_str])
                plotted_resistances_commanded.append(target_r)
                plotted_resistances_measured.append(measured_r_data)

                # --- Live Plot Update ---
                line.set_data(plotted_resistances_commanded, plotted_resistances_measured)
                
                # Dynamic X-axis adjustment
                ax.set_xlim(min(plotted_resistances_commanded) * 0.9, max(plotted_resistances_commanded) * 1.1)
                
                # Dynamic Y-axis adjustment for measured resistance
                if plotted_resistances_measured:
                    y_min = min(plotted_resistances_measured)
                    y_max = max(plotted_resistances_measured)
                    y_range = y_max - y_min
                    padding = y_range * 0.1 if y_range > 0 else 0.1 
                    # Ensure minimum is not above max + padding for very small ranges
                    adjusted_y_min = y_min - padding
                    adjusted_y_max = y_max + padding
                    if adjusted_y_min >= adjusted_y_max: # Handle cases where range is almost zero
                         adjusted_y_min = y_min - 0.1
                         adjusted_y_max = y_max + 0.1
                    ax.set_ylim(adjusted_y_min, adjusted_y_max)


                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.01) # Short pause to allow plot to render
            else:
                print(f"  Target Resistance {target_r:.2f} Ohms: Failed to get resistance measurement. Recording 'ERROR'. Settings: {knob_settings_applied_str}") 
                measurement_data.append([target_r, "ERROR", knob_settings_applied_str])

        print("\n--- All Targeted Resistance Setting & Resistance Measurement Sequence Complete ---") 
        send_gcode(printer_ser, "G1 Z0 F1000") 
        send_gcode(printer_ser, "M400")
        send_gcode(printer_ser, "M18")       

        print(f"\nSaving all commanded settings and measured resistances to {OUTPUT_CSV_FILE}...") 
        with open(OUTPUT_CSV_FILE, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerows(measurement_data)
        print(f"Data saved successfully. You can find the data in '{OUTPUT_CSV_FILE}'.")

        # --- Final Plot Saving ---
        if plotted_resistances_commanded and plotted_resistances_measured:
            print(f"\nSaving final plot to '{PLOT_OUTPUT_FILE}'...")
            plt.ioff() # Turn off interactive mode
            plt.tight_layout() # Adjust layout to prevent labels overlapping
            plt.savefig(PLOT_OUTPUT_FILE)
            print(f"Plot saved successfully to '{PLOT_OUTPUT_FILE}'.")
            plt.show() # Display the final plot and keep it open
        else:
            print("\nSkipping plot generation: No successful resistance measurements to plot.") 
            plt.ioff() # Ensure interactive mode is off even if no plot

        print(f"\n--- Summary of Commanded Resistance Settings and Measured Resistances for Range " 
              f"({min_desired_resistance} - {max_desired_resistance} Ohms, step {desired_resistance_step}) ---")
        
        summary_rows = [row for row in measurement_data[1:]]

        if not summary_rows:
            print("No data was recorded for the summary.")
        else:
            for row in summary_rows:
                target_r, measured_r_data, knob_settings = row 
                
                if measured_r_data == "ERROR":
                    print(f"  Target Resistance {float(target_r):.2f} Ohms: Measured Resistance: Measurement Failed. Settings: {knob_settings}") 
                else:
                    print(f"  Target Resistance {float(target_r):.2f} Ohms: Measured Resistance {float(measured_r_data):.4f} Ohms. Settings: {knob_settings}") 

    except serial.SerialException as e:
        print(f"Connection Error: {e}")
        print("Please check:\n- Correct **COM ports** and **baud rates** for your printer and multimeter.\n- Both devices are **connected and powered on**.\n- No other software is using these serial ports (e.g., OctoPrint, Cura, multimeter software).")
    except Exception as e:
        print(f"An unexpected error occurred during script execution: {e}")
    finally:
        if printer_ser is not None and printer_ser.is_open:
            printer_ser.close()
            print("Printer serial port closed.")
        if multimeter_ser is not None and multimeter_ser.is_open:
            multimeter_ser.close()
            print("Multimeter serial port closed.")
