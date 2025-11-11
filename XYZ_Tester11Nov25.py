import serial
import re # Import regex module for robust stripping
import csv
import math # For math.isclose for float comparisons
import sys # Import the sys module for sys.exit()
import matplotlib.pyplot as plt # Import for plotting
import numpy as np # Import for numerical operations, specifically np.nan for missing data
import pyvisa
import time
from multimeter_driver import read_3458a_resistance, send_gcode

# --- Global Serial Port Objects ---
# These will be initialized in the __main__ block
printer_ser = None

# --- Global State for Extruder Position Tracking ---
# This dictionary will store the last E-value commanded for each (X, Y) knob coordinate.
# It persists across different target_total_r iterations.
last_e_positions = {}

# --- Global State for Printer's Last Known XY Position ---
# This will track the actual (X,Y) coordinates the printer physically moved to last.
# This helps optimize Z lifts/lowers for consecutive operations on the same knob.
current_printer_xy_pos = (None, None)

# --- Global State for Printer's Last Known E Position ---
# This will track the actual E-axis coordinate the printer physically moved to last.
current_printer_e_pos = 0.0 # Initialize to 0.0, assuming E-axis starts at 0 or is homed to 0.

# --- Measurement Configuration (continued) ---
NUM_READINGS_PER_POINT = 2 # Number of readings to take at each knob position
# Define a small tolerance for robust floating-point comparisons/divisions
FLOATING_POINT_TOLERANCE = 1e-9

hp_3458a_address = 'GPIB0::22::INSTR'

# --- Special Cases Map: target -> decade to skip ---
SPECIAL_CASES = {
    0.1: 0.1,
    1.0: 1,
    10.0: 10,
    100.0: 100,
    1000.0: 1000
}

def get_e_for_knob_position(knob_position_int, extruder_e_values):
    """
    Maps an integer knob position (0-10) to its corresponding E-axis value.
    Args:
        knob_position_int (int): The desired knob setting, from 0 to 10.
        extruder_e_values (list): List of possible E values (e.g., [0, 4, ..., 40]).
    Returns:
        float: The calculated E-value for the knob.
    """
    if 0 <= knob_position_int < len(extruder_e_values):
        return float(extruder_e_values[knob_position_int])
    else:
        print(f"Warning: Knob position {knob_position_int} is out of the 0-{len(extruder_e_values)-1} range. Defaulting to E0.")
        return float(extruder_e_values[0])

def _calculate_knob_settings(target_total_r, all_knob_definitions, extruder_e_values, decades_to_skip=None):
    """
    Base function to calculate physical settings for the decade knobs.
    This is the pure calculation engine from the test script.
    """
    print(f"     Calculating physical knob settings for target total resistance: {target_total_r:.2f} Ohms...")

    # Sort knobs in descending order of decade value to perform greedy allocation
    sorted_knobs_desc = sorted(all_knob_definitions, key=lambda p: p['decade_value'], reverse=True)

    target_knob_e_settings = {(p['coords'][0], p['coords'][1]): 0.0 for p in all_knob_definitions}
    remaining_r_to_set = max(0.0, target_total_r)

    for knob_info in sorted_knobs_desc:
        knob_coords = (knob_info['coords'][0], knob_info['coords'][1])
        knob_label = knob_info['label']
        decade_step_value = knob_info['decade_value']

        if decades_to_skip and decade_step_value in decades_to_skip:
            print(f"      {knob_label}: Special case skip. Remaining: {remaining_r_to_set:.3f} Ohms")
            # If skipping, ensure the knob is set to 0
            target_knob_e_settings[knob_coords] = get_e_for_knob_position(0, extruder_e_values)
            continue

        # Use integer division to find the knob setting (0-10)
        # FIX: Add FLOATING_POINT_TOLERANCE to prevent math.floor() errors 
        # for numbers that are just under an integer (e.g., 6.999...)
        knob_setting = math.floor(remaining_r_to_set / decade_step_value + FLOATING_POINT_TOLERANCE)
        
        # Clamp the setting to the valid 0-10 range for the physical knob
        clamped_knob_setting = min(10, max(0, int(knob_setting)))

        # Calculate the actual resistance contributed by this knob
        actual_r_contribution = clamped_knob_setting * decade_step_value

        e_to_set = get_e_for_knob_position(clamped_knob_setting, extruder_e_values)
        target_knob_e_settings[knob_coords] = e_to_set
        remaining_r_to_set -= actual_r_contribution
        remaining_r_to_set = max(0.0, remaining_r_to_set)

        print(f"      {knob_label} (step {decade_step_value} Ohm): Setting to {clamped_knob_setting} "
              f"({actual_r_contribution:.3f} Ohms). E-value: {e_to_set:.2f}. "
              f"Remaining: {remaining_r_to_set:.3f} Ohms")

    return target_knob_e_settings

def set_all_decade_knobs(printer_ser, knob_e_settings, all_knob_definitions, extruder_e_values, extruder_feedrate, path_feedrate, last_e_positions_ref, working_z_height):
    """
    Applies the physical settings for the decade knobs based on a pre-calculated settings dictionary.
    This refactored function ONLY handles physical movements and updates.
    """
    global current_printer_xy_pos
    global current_printer_e_pos

    moves_to_execute = []
    for knob_coords, new_e_value in knob_e_settings.items():
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

        if (x_coord, y_coord) != current_printer_xy_pos:
            print(f"      [DEBUG] Lifting Z to Z0 before moving to {knob_label}...")
            send_gcode(printer_ser, "G1 Z0 F2000")
            send_gcode(printer_ser, "M400")

            send_gcode(printer_ser, "M18 E")
            send_gcode(printer_ser, "M400")

            print(f"      [DEBUG] Moving to X:{x_coord:.2f}, Y:{y_coord:.2f} for {knob_label}...")
            send_gcode(printer_ser, f"G1 X{x_coord} Y{y_coord} F{path_feedrate}")
            send_gcode(printer_ser, "M400")
            current_printer_xy_pos = (x_coord, y_coord)
            time.sleep(0.5)

            send_gcode(printer_ser, "M17 E")
            send_gcode(printer_ser, "M400")
            print(f"      [DEBUG] Setting E to its previous state E:{old_e_value:.2f} for {knob_label}...")
            send_gcode(printer_ser, f"G1 E{old_e_value:.2f} F{extruder_feedrate}")
            send_gcode(printer_ser, "M400")
            current_printer_e_pos = old_e_value
            time.sleep(0.2)

            print(f"      [DEBUG] Lowering Z to Z{working_z_height} for {knob_label}...")
            send_gcode(printer_ser, f"G1 Z{working_z_height} F2000")
            send_gcode(printer_ser, "M400")
            time.sleep(0.2)
        else:
            print(f"      [DEBUG] Staying at {knob_label} (X:{x_coord:.2f}, Y:{y_coord:.2f}) to adjust E-axis.")
            send_gcode(printer_ser, f"G1 Z{working_z_height} F2000")
            send_gcode(printer_ser, "M400")
            time.sleep(0.1)

        print(f"      [DEBUG] Setting E to its new state E:{new_e_value:.2f} for {knob_label}...")
        send_gcode(printer_ser, f"G1 E{new_e_value:.2f} F{extruder_feedrate}")
        send_gcode(printer_ser, "M400")
        current_printer_e_pos = new_e_value
        time.sleep(0.5)

        last_e_positions_ref[knob_coords] = new_e_value
        knob_settings_summary_list.append(f"{knob_label}:E{new_e_value:.2f}")
    
    return ", ".join(knob_settings_summary_list)

# Printer Settings
PRINTER_PORT = 'COM4'
PRINTER_BAUDRATE = 250000

# CSV Output File
OUTPUT_CSV_FILE = 'resistance_measurements.csv'
PLOT_OUTPUT_FILE = 'resistance_plot.png'

# Define XY Coordinates for each Decade Knob on the 1491G and their Expected Inductance Ranges
path_points_1491G = [
    {'coords': (1, 140),     'label': '0.1 mH steps', 'decade_value': 0.1, 'expected_min_r': 0, 'expected_max_r': 1.0},
    {'coords': (67, 121),  'label': '1 mH Decade',      'decade_value': 1,  'expected_min_r': 0, 'expected_max_r': 10},
    {'coords': (135, 105), 'label': '10 mH Decade',         'decade_value': 10,     'expected_min_r': 0, 'expected_max_r': 100},
    {'coords': (200, 86), 'label': '100 mH Decade',    'decade_value': 100,    'expected_min_r': 0, 'expected_max_r': 1000},
]

# Define XY Coordinates for each Decade Knob on the 1433F and their Expected Resistance Ranges
path_points_1433F = [
    {'coords': (140, 172),    'label': '0.01 Ohm Decade', 'decade_value': 0.01, 'expected_min_r': 0, 'expected_max_r': 0.1},
    {'coords': (61, 144), 'label': '0.1 Ohm Decade',  'decade_value': 0.1,    'expected_min_r': 0, 'expected_max_r': 1.0},
    {'coords': (140, 118),  'label': '1 Ohm Decade',  'decade_value': 1,          'expected_min_r': 0, 'expected_max_r': 10},
    {'coords': (61, 90), 'label': '10 Ohm Decade',    'decade_value': 10,    'expected_min_r': 0, 'expected_max_r': 100},
    {'coords': (140, 64),'label': '100 Ohm Decade',  'decade_value': 100, 'expected_min_r': 0, 'expected_max_r': 1000},
    {'coords': (61, 36),'label': '1K Ohm Decade',    'decade_value': 1000, 'expected_min_r': 0, 'expected_max_r': 10000},
    {'coords': (140, 10),'label': '10K Ohm Decade',  'decade_value': 10000, 'expected_min_r': 0, 'expected_max_r': 100000},
]

path_feedrate = 2000
extruder_feedrate = 2000

if __name__ == "__main__":
    measurement_data = []
    plotted_resistances_commanded = []
    plotted_resistances_measured_mean = []
    plotted_std_devs = []

    selected_instrument = None
    working_z_height = None
    while selected_instrument not in ['1', '2']:
        print("\n--- Instrument Selection ---")
        print("1. General Radio 1491G (Inductance Decade Box - if configured for resistance measurement)")
        print("2. General Radio 1433F (Resistance Decade Box)")
        selection_input = input("Enter 1 or 2: ").strip()

        if selection_input == '1':
            selected_instrument = '1'
            path_points = path_points_1491G
            working_z_height = 8
            print("Selected: General Radio 1491G")
        elif selection_input == '2':
            selected_instrument = '2'
            path_points = path_points_1433F
            working_z_height = 11
            print("Selected: General Radio 1433F")
        else:
            print("Invalid selection. Please enter '1' or '2'.")

    if selected_instrument == '1':
        extruder_e_values = list(range(0, 41, 4))
        print(f"Extruder E-values set for 1491G: {extruder_e_values}")
    else:
        extruder_e_values = list(range(0, 41, 4))
        print(f"Extruder E-values set for 1433F: {extruder_e_values}")

    min_desired_resistance = None
    max_desired_resistance = None
    desired_resistance_step = None

    while min_desired_resistance is None:
        try:
            min_input = input("Enter the minimum desired value to set on the decade box: ")
            min_desired_resistance = float(min_input)
        except ValueError:
            print("Invalid input. Please enter a numerical value for minimum resistance.")
            
    while max_desired_resistance is None:
        try:
            max_input = input("Enter the maximum desired value to set on the decade box: ")
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
    print(f"For each set resistance, {NUM_READINGS_PER_POINT} DC resistance readings will be taken from the multimeter.")
    print(f"The mean and standard deviation of these readings will be calculated.")
    print(f"The commanded knob settings, mean measured resistances, and standard deviations will be recorded to '{OUTPUT_CSV_FILE}'.")
    print(f"A plot of Commanded Resistance vs. Mean Measured Resistance (with Std Dev) will be saved to '{PLOT_OUTPUT_FILE}'.")

    plt.ion()
    fig, ax1 = plt.subplots(figsize=(10, 6))

    line_res, = ax1.plot([], [], marker='o', linestyle='-', color='blue', label='Mean Measured Resistance')
    
    ax1.set_title('Commanded Resistance vs. Measured Resistance & Std Dev (Live)')
    ax1.set_xlabel('Commanded Resistance (Ohms)')
    ax1.set_ylabel('Mean Measured Resistance (Ohms)', color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1.grid(True)
    
    ax2 = ax1.twinx()
    line_std_dev, = ax2.plot([], [], marker='x', linestyle='--', color='red', label='Standard Deviation')
    ax2.set_ylabel('Standard Deviation (Ohms)', color='red')
    ax2.tick_params(axis='y', labelcolor='red')

    lines = [line_res, line_std_dev]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper left')

    try:
        printer_ser = serial.Serial(PRINTER_PORT, PRINTER_BAUDRATE, timeout=5)
        print(f"\nConnected to printer on {PRINTER_PORT} at {PRINTER_BAUDRATE} baud.")

        printer_ser.reset_input_buffer()
        printer_ser.reset_output_buffer()
        
        send_gcode(printer_ser, "G90")
        send_gcode(printer_ser, "M106 S255")
        send_gcode(printer_ser, "G28 Z")
        send_gcode(printer_ser, "G28 X")
        send_gcode(printer_ser, "G28 Y")
        send_gcode(printer_ser, "M17")
        send_gcode(printer_ser, "M400")
        send_gcode(printer_ser, "M114")
        
        current_printer_xy_pos = (0.0, 0.0)
        current_printer_e_pos = 0.0

        send_gcode(printer_ser, "M18 E")
        send_gcode(printer_ser, "M400")
        print("Extruder stepper disabled to prevent heating when not in use.")

        print("\n--- Starting Targeted Resistance Setting & Resistance Measurement Sequence ---")
        
        # Create the list of target resistances with a small tolerance for floating point
        target_resistances_to_find = np.arange(min_desired_resistance, max_desired_resistance + desired_resistance_step * 0.001, desired_resistance_step).tolist()

        for i, target_r in enumerate(target_resistances_to_find):
            print(f"\n--- Iteration {i+1}/{len(target_resistances_to_find)}: Setting target resistance to {target_r:.3f} Ohms ---")

            special_match = None
            for val in SPECIAL_CASES.keys():
                if math.isclose(target_r, val):
                    special_match = val
                    break

            if special_match is not None:
                print(f"  --- Special Handling for {target_r:.3f} Ohms: Measuring both methods ---")
                
                skip_decade = SPECIAL_CASES[special_match]
                
                # --- Method 1: Skipping decade ---
                print("  --- Calculating and Applying Method 1 (Skipping Decade) ---")
                knob_e_settings_special = _calculate_knob_settings(target_r, path_points, extruder_e_values, decades_to_skip=[skip_decade])
                knob_settings_summary_special = set_all_decade_knobs(
                    printer_ser,
                    knob_e_settings_special,
                    path_points,
                    extruder_e_values,
                    extruder_feedrate,
                    path_feedrate,
                    last_e_positions,
                    working_z_height
                )
                
                current_knob_readings_special = []
                print(f"      Taking {NUM_READINGS_PER_POINT} readings for Method 1...")
                for _ in range(NUM_READINGS_PER_POINT):
                    measured_r = read_3458a_resistance(hp_3458a_address)
                    if measured_r is not None:
                        current_knob_readings_special.append(measured_r)
                    time.sleep(0.5)

                if current_knob_readings_special:
                    mean_measured_r_special = np.mean(current_knob_readings_special)
                    std_dev_measured_r_special = np.std(current_knob_readings_special)
                    print(f"      Method 1 Mean Measured Resistance: {mean_measured_r_special:.3f} Ohms")
                    print(f"      Method 1 Standard Deviation: {std_dev_measured_r_special:.3e} Ohms")
                    
                    measurement_data.append([f"Method 1: {target_r:.3f}", mean_measured_r_special, std_dev_measured_r_special, knob_settings_summary_special, current_knob_readings_special])
                else:
                    print("      No valid readings obtained for Method 1.")

                # --- Method 2: Standard allocation ---
                print("\n  --- Calculating and Applying Method 2 (Standard Allocation) ---")
                knob_e_settings_standard = _calculate_knob_settings(target_r, path_points, extruder_e_values)
                knob_settings_summary_standard = set_all_decade_knobs(
                    printer_ser,
                    knob_e_settings_standard,
                    path_points,
                    extruder_e_values,
                    extruder_feedrate,
                    path_feedrate,
                    last_e_positions,
                    working_z_height
                )

                current_knob_readings_standard = []
                print(f"      Taking {NUM_READINGS_PER_POINT} readings for Method 2...")
                for _ in range(NUM_READINGS_PER_POINT):
                    measured_r = read_3458a_resistance(hp_3458a_address)
                    if measured_r is not None:
                        current_knob_readings_standard.append(measured_r)
                    time.sleep(0.5)

                if current_knob_readings_standard:
                    mean_measured_r_standard = np.mean(current_knob_readings_standard)
                    std_dev_measured_r_standard = np.std(current_knob_readings_standard)
                    print(f"      Method 2 Mean Measured Resistance: {mean_measured_r_standard:.3f} Ohms")
                    print(f"      Method 2 Standard Deviation: {std_dev_measured_r_standard:.3e} Ohms")

                    measurement_data.append([f"Method 2: {target_r:.3f}", mean_measured_r_standard, std_dev_measured_r_standard, knob_settings_summary_standard, current_knob_readings_standard])
                else:
                    print("      No valid readings obtained for Method 2.")

                # We can't plot both methods on a single point easily, so we'll just plot the standard method here for the live graph.
                # The CSV will have both.
                plotted_resistances_commanded.append(target_r)
                plotted_resistances_measured_mean.append(mean_measured_r_standard)
                plotted_std_devs.append(std_dev_measured_r_standard)

            else:
                # Standard case (not a special number)
                knob_e_settings_standard = _calculate_knob_settings(target_r, path_points, extruder_e_values)
                knob_settings_summary = set_all_decade_knobs(
                    printer_ser,
                    knob_e_settings_standard,
                    path_points,
                    extruder_e_values,
                    extruder_feedrate,
                    path_feedrate,
                    last_e_positions,
                    working_z_height
                )
                
                current_knob_readings = []
                print(f"      Taking {NUM_READINGS_PER_POINT} readings for this position...")
                for _ in range(NUM_READINGS_PER_POINT):
                    measured_r = read_3458a_resistance(hp_3458a_address)
                    if measured_r is not None:
                        current_knob_readings.append(measured_r)
                    else:
                        print("      Warning: Failed to get a reading from multimeter.")
                    time.sleep(0.5)

                if current_knob_readings:
                    mean_measured_r = np.mean(current_knob_readings)
                    std_dev_measured_r = np.std(current_knob_readings)
                    print(f"      Mean Measured Resistance: {mean_measured_r:.3f} Ohms")
                    print(f"      Standard Deviation: {std_dev_measured_r:.3e} Ohms")
                else:
                    mean_measured_r = np.nan
                    std_dev_measured_r = np.nan
                    print("      No valid readings obtained for this position.")
                
                measurement_data.append([target_r, mean_measured_r, std_dev_measured_r, knob_settings_summary, current_knob_readings])
                plotted_resistances_commanded.append(target_r)
                plotted_resistances_measured_mean.append(mean_measured_r)
                plotted_std_devs.append(std_dev_measured_r)

            # Update plot
            line_res.set_data(plotted_resistances_commanded, plotted_resistances_measured_mean)
            line_std_dev.set_data(plotted_resistances_commanded, plotted_std_devs)

            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()
            
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(1)

    except serial.SerialException as e:
        print(f"Serial Port Error: {e}")
        print("Please ensure the correct COM ports are selected and devices are connected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        
        print("\n--- Measurement sequence finished or interrupted. ---")

        if printer_ser and printer_ser.is_open:
            
            # --- NEW CODE BLOCK: RESTORE KNOBS TO E0 ---
            print("\n--- Restoring all decade box knobs to 0 (E0) position ---")
            
            # 1. Define the E=0 setting for all knobs
            e0_value = extruder_e_values[0] # Should be 0.0
            knob_e_settings_reset = {
                (p['coords'][0], p['coords'][1]): float(e0_value) for p in path_points
            }
            
            # 2. Use the set_all_decade_knobs function to move the extruder to E0 for every knob.
            # This efficiently handles Z-lifts and XY movement between knobs.
            reset_summary = set_all_decade_knobs(
                printer_ser,
                knob_e_settings_reset,
                path_points,
                extruder_e_values,
                extruder_feedrate,
                path_feedrate,
                last_e_positions, # This will update last_e_positions to all E0
                working_z_height
            )
            print(f"Knobs reset summary: {reset_summary}")
            
            # --- END NEW CODE BLOCK ---
            
            print("Moving Z to Z0 and XY to origin (0,0)...")
            send_gcode(printer_ser, "G1 Z0 F2000")
            send_gcode(printer_ser, "M400")
            send_gcode(printer_ser, f"G1 X0 Y0 F{path_feedrate}")
            send_gcode(printer_ser, "M400")
            current_printer_xy_pos = (0.0, 0.0)
            print("Disabling all steppers.")
            send_gcode(printer_ser, "M18")
            send_gcode(printer_ser, "M400")
            send_gcode(printer_ser, "M107")
            
            if printer_ser and printer_ser.is_open:
                printer_ser.close()
                print(f"Printer serial port {PRINTER_PORT} closed.")
                
        if measurement_data:
            try:
                with open(OUTPUT_CSV_FILE, 'w', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(['Target_Resistance_Ohms', 'Measured_Resistance_Mean_Ohms', 'Measured_Resistance_StdDev_Ohms', 'Knob_Settings_Applied', 'Individual_Readings'])
                    
                    for row in measurement_data:
                        target_r, mean_r, std_dev, knob_settings, readings = row
                        readings_str = ', '.join(f'{r:.4f}' for r in readings)
                        csv_writer.writerow([target_r, mean_r, std_dev, knob_settings, readings_str])
                        
                print(f"Measurement data saved to '{OUTPUT_CSV_FILE}'.")
            except IOError as e:
                print(f"Error writing CSV file: {e}")
        else:
            print("No measurement data to save.")

        if plotted_resistances_commanded:
            try:
                plt.ioff()
                fig.savefig(PLOT_OUTPUT_FILE)
                print(f"Plot saved to '{PLOT_OUTPUT_FILE}'.")
            except Exception as e:
                print(f"Error saving plot: {e}")
        else:
            print("No data to plot.")
            
        
        plt.close(fig)
        sys.exit(0)