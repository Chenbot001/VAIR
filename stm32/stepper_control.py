import serial
import time

# --- Configuration ---
# Find your STM32's COM port in Device Manager or Arduino IDE
# On Windows it's 'COMx', on Linux/macOS it's '/dev/ttyACMx' or '/dev/tty.usbmodemx'
SERIAL_PORT = 'COM5'  # <--- CHANGE THIS to your STM32's port
BAUD_RATE = 115200

def send_command(serial_conn, m1_pos, m2_pos, freq, microsteps):
    """Formats and sends a command to the STM32."""
    # Format the command string as expected by the microcontroller
    command = f"<{m1_pos},{m2_pos},{freq},{microsteps}>\n"
    print(f"Sending command: {command.strip()}")
    # Send the command as bytes
    serial_conn.write(command.encode('utf-8'))

def main():
    """Main function to run the interactive controller."""
    try:
        # Establish serial connection
        stm32 = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2) # Increased timeout slightly
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        time.sleep(2)  # Wait for the STM32 to reset and initialize

        # Clear any initial messages from the buffer
        if stm32.in_waiting > 0:
            print(stm32.read(stm32.in_waiting).decode('utf-8', errors='ignore').strip())

        # Interactive loop to get commands from the user
        while True:
            print("\nEnter new motor targets (or Ctrl + C to quit):")
            try:
                m1 = int(input("  Enter Motor 1 Target Position (full steps): "))
                m2 = int(input("  Enter Motor 2 Target Position (full steps): "))
                hz = float(input("  Enter Frequency (Hz): "))
                ms = int(input("  Enter Microstep Factor (e.g., 4, 8, 16): "))

                send_command(stm32, m1, m2, hz, ms)
                # Wait for and print the response from the STM32
                # --- MODIFIED LINE ---
                # Added errors='ignore' to the decode call to prevent crashes from noisy data.
                response = stm32.read_until(b'Move complete. Motors disabled.\n').decode('utf-8', errors='ignore')
                
                print("--- STM32 Response ---")
                print(response.strip())
                print("----------------------")

            except ValueError:
                print("Invalid input. Please enter numbers only.")
            except KeyboardInterrupt:
                print("\nExiting...")
                break

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}.")
        print(e)
    finally:
        if 'stm32' in locals() and stm32.is_open:
            stm32.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()