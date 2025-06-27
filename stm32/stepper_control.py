# File: stepper_control.py

import serial
import time

# --- Configuration ---
# Set to the COM port of your Arduino UNO.
# Your last script used 'COM7', so I have set it here. Please verify this.
SERIAL_PORT = 'COM7' 
BAUD_RATE = 115200
MICROSTEPS = 16  # Fixed microstepping factor

def get_user_input():
    """Gets and validates user input for a motor command."""
    while True:
        try:
            m1_pos = int(input("  Enter Motor 1 Target Position (full steps): "))
            m2_pos = int(input("  Enter Motor 2 Target Position (full steps): "))
            freq = int(input("  Enter Frequency (full steps/sec, e.g., 1000): "))
            return m1_pos, m2_pos, freq
        except ValueError:
            print("Invalid input. Please enter integers only.")
        except KeyboardInterrupt:
            raise # Re-raise the exception to be caught by the main loop's handler

def send_motor_command(controller, m1_pos, m2_pos, freq):
    """Send a motor command and wait for completion."""
    command = f"<{m1_pos},{m2_pos},{freq},{MICROSTEPS}>\n"
    print(f"Sending command: {command.strip()}")
    
    # Send the command to the Arduino
    controller.write(command.encode('utf-8'))
    
    # Wait for the "Move complete" confirmation and print the response
    response = controller.read_until(b'Move complete. Motors disabled.\n').decode('utf-8', errors='ignore')
    print(f"Arduino response: {response.strip()}\n")

def main_controller():
    """Main function to run the interactive controller."""
    # The 'uno' variable will hold our serial connection
    uno = None 
    try:
        # Connect to the serial port with a 20-second timeout for the homing sequence
        uno = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=20)
        print(f"Connected to Arduino on {SERIAL_PORT} at {BAUD_RATE} baud.")

        # --- 1. Automated Homing Handshake ---
        print("\nWaiting for Arduino to be ready for homing...")
        # Wait for the "Ready" message from the Arduino's setup() function
        initial_message = uno.read_until(b'begin homing sequence.\n').decode('utf-8', errors='ignore')
        print(f"Arduino says: {initial_message.strip()}")

        # Send the <HOME> command to start the sequence
        print("Sending <HOME> command to start homing...")
        uno.write(b'<HOME>\n')

        # Wait for the entire homing sequence to finish
        print("Homing in progress... (This may take a moment)")
        homing_response = uno.read_until(b'Ready for motion commands.\n').decode('utf-8', errors='ignore')
        print("--- Homing Sequence Output ---")
        print(homing_response.strip())
        print("------------------------------\n")
        
        # --- 2. Initialization Step ---
        print("Performing initialization: moving both motors to position 500 at 500Hz...")
        send_motor_command(uno, 500, 500, 500)
        print("Initialization complete.\n")
        
        # --- 3. Main Command Loop ---
        while True:
            print("Enter new motor targets (or Ctrl + C to quit):")
            m1, m2, hz = get_user_input()
            send_motor_command(uno, m1, m2, hz)

    except serial.SerialException as e:
        print(f"\nError: Could not open or read from serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        # --- 4. Reset Step ---
        if uno and uno.is_open:
            try:
                print("Performing reset: moving both motors back to position 0...")
                send_motor_command(uno, 0, 0, 500)
                print("Reset complete.")
            except Exception as e:
                print(f"Error during reset: {e}")
            finally:
                # Ensure the serial port is closed when the program ends
                uno.close()
                print("Serial port closed.")

if __name__ == "__main__":
    main_controller()