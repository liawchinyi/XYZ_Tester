import serial
import csv
import sys
import time
import numpy as np
import pyvisa
import matplotlib.pyplot as plt
import pandas as pd # <-- NEW IMPORT
# Assuming multimeter_driver.py has read_3458a_resistance and send_gcode
from multimeter_driver import read_3458a_resistance, send_gcode

printer_ser = None

# --- Global State for Printer's Last Known XY Position ---
current_printer_xy_pos = (None, None)

# --- Global State for Printer's Last Known E Position ---
current_printer_e_pos = 0.0

# --- Measurement Configuration ---
NUM_READINGS_PER_POINT = 10 # Number of readings to take at each E position
# Define the specific E positions to test (0, 4, 8, ..., 40)
TEST_E_POSITIONS = list(range(0, 41, 4)) 

hp_3458a_address = 'GPIB0::22::INSTR'

# Printer Settings
PRINTER_PORT = 'COM4' #'/dev/ttyUSB0'
PRINTER_BAUDRATE = 250000

# CSV Output File
OUTPUT_EXCEL_FILE = 'GR1433F_Resistance.xlsx' # <-- NEW
PLOT_OUTPUT_FILE = 'GR1433F_Resistance.png'

# Define Decade Knob Coordinates 
path_points_1433F = [
    {'coords': (140, 172), 'label': '0.01 Ohm Decade', 'decade_value': 0.01},
    {'coords': (61, 144), 'label': '0.1 Ohm Decade', 'decade_value': 0.1},
    {'coords': (140, 118), 'label': '1 Ohm Decade', 'decade_value': 1},
    {'coords': (61, 90), 'label': '10 Ohm Decade', 'decade_value': 10},
    {'coords': (140, 62),'label': '100 Ohm Decade', 'decade_value': 100},
    {'coords': (61, 34),'label': '1K Ohm Decade', 'decade_value': 1000},
    {'coords': (140, 8),'label': '10K Ohm Decade', 'decade_value': 10000},
]
path_points = path_points_1433F
working_z_height = 11
path_feedrate = 2000
extruder_feedrate = 2000

# --- Core Refactored Functions ---

def move_to_knob_position(printer_ser, x_coord, y_coord, z_height, path_feedrate):
    """Moves the printer head to the X/Y location of a specific knob, then lowers Z."""
    global current_printer_xy_pos
    
    # Lift Z for safe travel if moving to a new knob
    if (x_coord, y_coord) != current_printer_xy_pos:
        send_gcode(printer_ser, "G1 Z0 F2000") # Lift Z to clearance height
        send_gcode(printer_ser, "M400")
        
        # Move XY
        send_gcode(printer_ser, f"G1 X{x_coord} Y{y_coord} F{path_feedrate}")
        send_gcode(printer_ser, "M400")
        current_printer_xy_pos = (x_coord, y_coord)
    
    # Lower Z to working height
    send_gcode(printer_ser, f"G1 Z{z_height} F2000")
    send_gcode(printer_ser, "M400")
    time.sleep(0.1)

def set_e_position(printer_ser, target_e_value, extruder_feedrate):
    """Commands the E-axis to a specific position."""
    global current_printer_e_pos
    
    # Ensure E motor is enabled before movement
    send_gcode(printer_ser, "M17 E") 
    send_gcode(printer_ser, "M400")

    send_gcode(printer_ser, f"G1 E{target_e_value:.2f} F{extruder_feedrate}")
    send_gcode(printer_ser, "M400")
    current_printer_e_pos = target_e_value
    time.sleep(0.1)

def test_single_knob_range(printer_ser, knob_info, working_z_height, path_feedrate, extruder_feedrate, test_e_positions):
    """
    Tests a single decade knob across all E-axis positions, calculates mean/std dev, 
    and returns the data.
    """
    x_coord, y_coord = knob_info['coords']
    knob_label = knob_info['label']
    decade_value = knob_info['decade_value']
    knob_test_data = []

    print(f"\n--- Testing Knob: {knob_label} (Decade: {decade_value} Ohms) ---")
    
    # 1. Move to the knob's location and lower Z
    move_to_knob_position(printer_ser, x_coord, y_coord, working_z_height, path_feedrate)

    for e_value in test_e_positions:
        knob_setting = int(e_value / 4) # Assuming 0->E0, 1->E4, ..., 10->E40
        commanded_r = knob_setting * decade_value
        
        print(f"  Setting Knob to: {knob_setting} (E-axis: {e_value:.2f}). Commanded R: {commanded_r:.4f} Ohms...")
        
        # 2. Set the E position (turn the knob)
        set_e_position(printer_ser, e_value, extruder_feedrate)
        
        # 3. Take multiple readings
        current_readings = []
        for i in range(NUM_READINGS_PER_POINT):
            measured_r = read_3458a_resistance(hp_3458a_address)
            
            # --- CRITICAL HALT CONDITION ---
            if measured_r is None:
                print(f"\n[CRITICAL ERROR] Multimeter failed to return a valid reading for {knob_label} (Setting {knob_setting}).")
                print("Halting program execution to ensure data integrity.")
                
                # Perform essential cleanup before exiting
                if printer_ser and printer_ser.is_open:
                    send_gcode(printer_ser, "G1 Z0 F2000") # Lift Z for safety
                    send_gcode(printer_ser, "M18")       # Disable steppers
                    printer_ser.close()
                    
                # Exit the entire program with a non-zero exit code (1) to signal failure
                sys.exit(1) # <-- HALT HERE
            # -------------------------------
            
            current_readings.append(measured_r)
            time.sleep(0.5)

        # 4. Process and store data
        if current_readings:
            mean_measured_r = np.mean(current_readings)
            std_dev_measured_r = np.std(current_readings)
            
            # PPM Calculation (as added previously)
            if commanded_r != 0:
                ppm_error = ((mean_measured_r - commanded_r) / commanded_r) * 1e6
            else:
                ppm_error = 0.0

            knob_test_data.append({
                'Knob_Label': knob_label,
                'Decade_Value': decade_value,
                'Knob_Setting': knob_setting,
                'E_Position': e_value,
                'Commanded_R_Ohms': commanded_r,
                'Measured_R_Mean_Ohms': mean_measured_r,
                'Measured_R_StdDev_Ohms': std_dev_measured_r,
                'Measured_R_PPM_Error': ppm_error,
                'Individual_Readings': current_readings
            })
            print(f"    Measured: {mean_measured_r:.4f} Ohms (Std Dev: {std_dev_measured_r:.2e}, PPM: {ppm_error:.1f})")
        # The 'else' block for this 'if current_readings' is now unnecessary because the program halts if a reading fails.
            
    # 5. Reset Knob to E0
    print(f"  Resetting {knob_label} to E0.")
    set_e_position(printer_ser, 0.0, extruder_feedrate)
    
    # Disable E motor to prevent heat/power consumption while moving to next knob
    send_gcode(printer_ser, "M18 E")
    send_gcode(printer_ser, "M400")

    return knob_test_data

def plot_results(all_data, plot_file):
    """
    Generates and saves a plot showing the resistance error (Measured - Commanded) 
    and the measurement standard deviation for each decade knob setting.
    """
    # Organize data by knob (Decade Value)
    knob_data_map = {}
    for row in all_data:
        decade = row['Decade_Value']
        if decade not in knob_data_map:
            knob_data_map[decade] = []
        knob_data_map[decade].append(row)

    # Prepare plot structure
    fig, ax1 = plt.subplots(figsize=(14, 8))
    
    ax1.set_xlabel('Knob Setting (0 to 10)')
    ax1.set_ylabel('Resistance Error ($\Delta R = R_{Measured} - R_{Commanded}$) [Ohms]', color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax1.set_title(f'GR-1433F Decade Resistance Error and Uncertainty vs. Setting')
    
    # Secondary Y-axis for Standard Deviation (Uncertainty)
    ax2 = ax1.twinx()
    ax2.set_ylabel('Measurement Standard Deviation ($\sigma_R$) [Ohms]', color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')
    ax2.set_yscale('log') # Use log scale for standard deviation as it can span many orders of magnitude
    
    # Define a color scheme for the different decade lines
    viridis_cmap = plt.colormaps.get_cmap('viridis')
    colors = viridis_cmap(np.linspace(0, 1, len(knob_data_map)))
    
    i = 0
    
    for decade, data_list in knob_data_map.items():
        # Sort data by Knob_Setting for plotting consistency
        data_list.sort(key=lambda x: x['Knob_Setting'])
        
        knob_settings = [d['Knob_Setting'] for d in data_list]
        commanded_r = np.array([d['Commanded_R_Ohms'] for d in data_list])
        measured_mean = np.array([d['Measured_R_Mean_Ohms'] for d in data_list])
        std_dev = np.array([d['Measured_R_StdDev_Ohms'] for d in data_list])

        # Calculate Error
        error = measured_mean - commanded_r
        
        # Format label to show decade value clearly (e.g., 100 $\Omega$)
        label_text = f'{decade:g} $\Omega$ Decade'
        
        # Plot Error on Primary Axis
        ax1.plot(knob_settings, error, marker='o', linestyle='-', 
                 color=colors[i], label=label_text) # Use colors[i] instead of colors(i)
        
        # Plot Standard Deviation on Secondary Axis
        # Filter out zeros for log scale visualization
        valid_std_dev = std_dev[std_dev > 0]
        valid_settings = np.array(knob_settings)[std_dev > 0]
        
        if len(valid_std_dev) > 0:
            # We use a scatter plot here to prevent multiple lines confusing the legend
            ax2.scatter(valid_settings, valid_std_dev, marker='x', color='tab:red', s=50, alpha=0.6)
        
        i += 1
        
    # Finalize plot and merge legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    # Add a dummy legend entry for the standard deviation points
    std_dev_label = ax2.scatter([], [], marker='x', color='tab:red', label='Std. Dev. ($\sigma_R$)')
    ax1.legend(lines1 + [std_dev_label], labels1 + ['Std. Dev. ($\sigma_R$)'], loc='lower left')
    
    plt.tight_layout()
    plt.savefig(plot_file)
    print(f"Plot saved to '{plot_file}'.")


# --- Main Execution Block ---
if __name__ == "__main__":
    
    all_measurement_data = []

    try:
        # --- Printer Initialization ---
        printer_ser = serial.Serial(PRINTER_PORT, PRINTER_BAUDRATE, timeout=5)
        print(f"\nConnected to printer on {PRINTER_PORT} at {PRINTER_BAUDRATE} baud.")

        printer_ser.reset_input_buffer()
        printer_ser.reset_output_buffer()
        
        send_gcode(printer_ser, "G90") # Absolute positioning
        send_gcode(printer_ser, "M106 S255") # Turn on warning light/fan 
        
        # Home Z first to prevent collision, then home all axes
        send_gcode(printer_ser, "G28 Z") 
        send_gcode(printer_ser, "G28") 
        
        send_gcode(printer_ser, "M17") # Enable all steppers
        send_gcode(printer_ser, "M400") # Wait for moves to finish
        send_gcode(printer_ser, "M114") # Get current position (initializes current_printer_xy_pos)
        current_printer_xy_pos = (0.0, 0.0)
        current_printer_e_pos = 0.0
        
        # Disable E stepper initially
        send_gcode(printer_ser, "M18 E")
        send_gcode(printer_ser, "M400")
        
        print("\n--- Starting Single Knob Characterization Sequence ---")

        # --- Sorting by Decade Value (0.01 Ohm first) ---
        path_points.sort(key=lambda p: p['decade_value'])
        print(f"Knob testing order set to: {[p['label'] for p in path_points]}")
        # -------------------------------------------------
        
        for knob_info in path_points:
            # Run the test for one knob, which handles all E positions and resets itself to E0
            knob_data = test_single_knob_range(
                printer_ser, 
                knob_info, 
                working_z_height, 
                path_feedrate, 
                extruder_feedrate, 
                TEST_E_POSITIONS
            )
            all_measurement_data.extend(knob_data)
            
    except KeyboardInterrupt:
        print("\n\n[USER INTERRUPT] Ctrl+C detected. Proceeding to safe shutdown and saving files...")
        
    except serial.SerialException as e:
        print(f"Serial Port Error: {e}")
        print("Please ensure the correct COM ports are selected and devices are connected.")
    
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        
    finally:
        
        print("\n--- Test sequence finished or interrupted. ---")

        # --- 1. Printer Cleanup ---
        if printer_ser and printer_ser.is_open:
            # Final safety movements
            send_gcode(printer_ser, "G1 Z0 F2000")
            send_gcode(printer_ser, "M400")
            send_gcode(printer_ser, f"G1 X0 Y0 F{path_feedrate}")
            send_gcode(printer_ser, "M400")
            send_gcode(printer_ser, "M18") # Disable all steppers
            send_gcode(printer_ser, "M400")
            send_gcode(printer_ser, "M107") # Disable warning lights/fan
            printer_ser.close()
            print(f"Printer serial port {PRINTER_PORT} closed.")

        # --- 2. Data Saving and Plotting (MOVED OUTSIDE 'if printer_ser...') ---
        if all_measurement_data:
            try:
                # 1a. Prepare data for DataFrame
                data_for_df = []
                for row in all_measurement_data:
                    # Create a copy to modify the Individual_Readings list
                    clean_row = row.copy() 
                    
                    # Convert the list of readings to a comma-separated string for Excel
                    clean_row['Individual_Readings'] = ', '.join(f'{r:.4f}' for r in clean_row['Individual_Readings'])
                    
                    # Remove the redundant E_Position key
                    if 'E_Position' in clean_row:
                        del clean_row['E_Position']
                        
                    data_for_df.append(clean_row)

                # 1b. Create and Save DataFrame to Excel
                df = pd.DataFrame(data_for_df)
                
                # Reorder columns to match the desired output order
                column_order = ['Knob_Label', 'Decade_Value', 'Knob_Setting', 
                                'Commanded_R_Ohms', 'Measured_R_Mean_Ohms', 'Measured_R_StdDev_Ohms',
                                'Individual_Readings']
                df = df[column_order]

                df.to_excel(OUTPUT_EXCEL_FILE, index=False) # index=False prevents writing the DataFrame index

                print(f"Measurement data saved to '{OUTPUT_EXCEL_FILE}'.")
                
                # --- 2. Generate Plot ---
                plot_results(all_measurement_data, PLOT_OUTPUT_FILE)
                
            except IOError as e:
                print(f"Error writing Excel/Plot file: {e}")
            except ImportError:
                # This error shouldn't occur now, but it's good to keep the check
                print("\n[ERROR] The 'pandas' or 'openpyxl' library is not installed.")
                print("Please install them using: pip install pandas openpyxl")
        else:
            print("No measurement data to save or plot.")
            
        sys.exit(0)
