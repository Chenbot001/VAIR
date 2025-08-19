import minimalmodbus
import time
import keyboard

# --- Configuration ---
PORT_NAME = 'COM11'
SLAVE_ADDRESS = 1
BAUDRATE = 9600
READ_REGISTER_ADDRESS = 0
ZERO_REGISTER_ADDRESS = 8      # Register for the zero command
DIRECTION_REGISTER_ADDRESS = 9 # Register for the direction command

# Constants for decoding
SINGLE_TURN_RESOLUTION = 2**15

def zero_encoder(instrument):
    """Sends the command to zero the encoder."""
    try:
        print("\nSending zeroing command...")
        # Command: 01 06 00 08 00 01 C9 C8
        instrument.write_register(
            registeraddress=ZERO_REGISTER_ADDRESS,
            value=1,
            functioncode=6
        )
        print("Zeroing command sent successfully!")
    except minimalmodbus.ModbusException as e:
        print(f"Failed to send zeroing command: {e}")

def set_direction(instrument, set_to_ccw):
    """
    Sends the command to set the encoder direction.
    - CW:  Write 0 to register 9 (01 06 00 09 00 00 59 C8)
    - CCW: Write 1 to register 9 (01 06 00 09 00 01 98 08)
    """
    try:
        value_to_write = 1 if set_to_ccw else 0
        direction_str = "Counter-Clockwise (CCW)" if set_to_ccw else "Clockwise (CW)"
        
        print(f"\nSetting direction to {direction_str}...")
        instrument.write_register(
            registeraddress=DIRECTION_REGISTER_ADDRESS,
            value=value_to_write,
            functioncode=6
        )
        print("Direction set successfully!")
        return set_to_ccw
    except minimalmodbus.ModbusException as e:
        print(f"Failed to set direction: {e}")
        # If the command fails, return the previous state
        return not set_to_ccw

def main():
    """Main function to connect to the encoder and read data."""
    # State variables for keyboard input
    z_key_was_pressed = False
    d_key_was_pressed = False
    is_ccw = False  # Start with default direction CW

    try:
        instrument = minimalmodbus.Instrument(PORT_NAME, SLAVE_ADDRESS)
        instrument.serial.baudrate = BAUDRATE
        instrument.serial.bytesize = 8
        instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
        instrument.serial.stopbits = 1
        instrument.serial.timeout = 0.5
        instrument.mode = minimalmodbus.MODE_RTU
        
        print(f"Successfully connected to encoder on {PORT_NAME}...")
        print("Press 'z' to zero | Press 'd' to toggle direction | Press Ctrl+C to stop.")
        
        # Set initial direction to CW
        set_direction(instrument, is_ccw)
        
        while True:
            # --- Handle Keyboard Input (Single press detection) ---

            # Zeroing ('z' key)
            if keyboard.is_pressed('z') and not z_key_was_pressed:
                zero_encoder(instrument)
                z_key_was_pressed = True
            elif not keyboard.is_pressed('z'):
                z_key_was_pressed = False

            # Direction Toggle ('d' key)
            if keyboard.is_pressed('d') and not d_key_was_pressed:
                is_ccw = set_direction(instrument, not is_ccw) # Toggle the state
                d_key_was_pressed = True
            elif not keyboard.is_pressed('d'):
                d_key_was_pressed = False

            # --- Read and Decode Encoder Data ---
            try:
                total_encoded_value = instrument.read_long(READ_REGISTER_ADDRESS, 3, False)
                single_turn_value = total_encoded_value & 0x7FFF
                turn_count = total_encoded_value >> 15
                angle = (single_turn_value / SINGLE_TURN_RESOLUTION) * 360.0
                
                direction_str = "CCW" if is_ccw else "CW"
                
                # --- MODIFIED LINE ---
                # Added Single-Turn and Raw values back to the output string
                print(
                    f"\rDir: {direction_str} | Angle: {angle:7.2f}° | Turns: {turn_count:3d} | "
                    f"Single-Turn: {single_turn_value:5d} | Raw: {total_encoded_value:7d}   ",
                    end=""
                )
                
                time.sleep(0.05) 

            except minimalmodbus.ModbusException as e:
                # print(f"\rModbus error: {e}" + " "*40)
                time.sleep(1)
            except Exception as e:
                print(f"\nAn error occurred: {e}")
                break

    except Exception as e:
        print(f"Failed to connect on {PORT_NAME}. Error: {e}")

if __name__ == "__main__":
    main()