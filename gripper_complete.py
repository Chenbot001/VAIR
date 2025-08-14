# File: gripper_complete.py
"""
Complete Gripper Control System
------------------------------
Integrated control for both stepper motor rotation and DM motor gripper actuation.
This script combines:
- Stepper motor control for gripper rotation (via Arduino)
- DM motor control for gripper open/close (via CAN)

Stepper Motor Controls:
- [A/D] Rotate gripper left/right
- [W/S] Adjust rotation speed
- [Q/E] Adjust step size

Gripper Controls:
- [O] Open gripper fully
- [C] Close gripper fully

General Controls:
- [SPACE] STOP all motors
- [R] Reset stepper position to center
- [ESC] Quit program
"""

import serial
import time
import os
import threading
import sys
import math

from pynput import keyboard

# Add paths for local imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'damiao', 'DM_Control'))


# Import DM motor control from the damiao folder
try:
    from DM_CAN import *
    DM_AVAILABLE = True
except ImportError:
    print("Warning: DM motor control not available. Gripper functions disabled.")
    DM_AVAILABLE = False



# --- Configuration ---
CONFIG = {
    # Stepper motor configuration
    "stepper_port": "COM9",
    "stepper_baud_rate": 115200,
    "microsteps": 16,
    "max_steps": 1000,
    "initial_pos": 500,
    "initial_speed": 100,
    
    # Gripper motor configuration
    "gripper_port": "COM3",
    "gripper_baud_rate": 921600,
    "gripper_motor_id": 0x01,
    "gripper_can_id": 0x11,
    

}

# Gripper position limits (in radians)
GRIPPER_MIN_POS = -1.35  # Fully closed position
GRIPPER_MAX_POS = 0.0    # Fully open position

# --- State Tracking ---
class SystemState:
    """A class to hold the live state of both motor controllers and sensor."""
    def __init__(self):
        # Stepper motor state
        self.m1_target_pos = CONFIG["initial_pos"]
        self.m2_target_pos = CONFIG["initial_pos"]
        self.speed = CONFIG["initial_speed"]
        self.step_size = 50
        
        # Gripper state
        self.gripper_closure_percent = 0  # 0% = open, 100% = closed
        self.gripper_position_rad = GRIPPER_MAX_POS
        
        # System state
        self.last_stepper_msg = ""
        self.last_gripper_msg = ""
        self.running = True
        self.stepper_connected = False
        self.gripper_connected = False

# Global state object
state = SystemState()

# --- Gripper Helper Functions ---
def percentage_to_position(percentage):
    """Convert closure percentage to radian position"""
    percentage = max(0, min(100, percentage))
    position = GRIPPER_MAX_POS + (percentage/100.0) * (GRIPPER_MIN_POS - GRIPPER_MAX_POS)
    return position

def position_to_percentage(position):
    """Convert radian position to closure percentage"""
    position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
    percentage = ((position - GRIPPER_MAX_POS) / (GRIPPER_MIN_POS - GRIPPER_MAX_POS)) * 100
    return percentage

def safe_gripper_position(position):
    """Ensures the gripper position stays within safe limits"""
    limited_position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
    if limited_position != position:
        state.last_gripper_msg = f"Position limited: {position:.3f} → {limited_position:.3f}"
    return limited_position



# --- Serial Communication ---
def stepper_reader(ser, state):
    """Continuously reads from the stepper serial port in the background."""
    while state.running:
        try:
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    state.last_stepper_msg = response
        except serial.SerialException:
            state.last_stepper_msg = "ERR: Stepper serial disconnected."
            state.stepper_connected = False
            break
        time.sleep(0.05)

def send_stepper_command(ser, command):
    """Sends a command to the Arduino."""
    if ser and ser.is_open:
        ser.write(command.encode('utf-8'))

def send_stepper_move_command(ser, m1, m2, speed):
    """Constructs and sends a standard stepper motor move command."""
    if not ser or not ser.is_open:
        return
    
    # Clamp position to physical limits as a safety measure
    m1 = max(0, min(m1, CONFIG["max_steps"]))
    m2 = max(0, min(m2, CONFIG["max_steps"]))
    command = f"<{int(m1)},{int(m2)},{int(speed)},{CONFIG['microsteps']}>\n"
    send_stepper_command(ser, command)
    
    # Update the state's target position
    state.m1_target_pos = m1
    state.m2_target_pos = m2

# --- Gripper Control Functions ---
def setup_gripper():
    """Initialize and configure the gripper motor"""
    if not DM_AVAILABLE:
        return None, None, None
    
    try:
        # Setup motor
        motor = Motor(DM_Motor_Type.DM4310, CONFIG["gripper_motor_id"], CONFIG["gripper_can_id"])
        
        # Setup serial connection
        serial_port = serial.Serial(CONFIG["gripper_port"], CONFIG["gripper_baud_rate"], timeout=0.5)
        motor_control = MotorControl(serial_port)
        
        # Add and enable motor
        motor_control.addMotor(motor)
        motor_control.enable(motor)
        
        # Set control mode
        motor_control.switchControlMode(motor, Control_Type.POS_VEL)
        
        state.gripper_connected = True
        state.last_gripper_msg = "Gripper motor enabled successfully"
        
        return motor, motor_control, serial_port
    
    except Exception as e:
        state.last_gripper_msg = f"ERR: Gripper setup failed: {e}"
        return None, None, None

def move_gripper_to_percentage(motor_control, motor, percentage, velocity=2.0):
    """Move gripper to a specific closure percentage"""
    if not motor_control or not motor:
        state.last_gripper_msg = "ERR: Gripper not available"
        return
    
    try:
        # Convert percentage to radians
        position = percentage_to_position(percentage)
        safe_pos = safe_gripper_position(position)
        
        # Update state
        state.gripper_closure_percent = percentage
        state.gripper_position_rad = safe_pos
        
        # Send command to motor
        motor_control.control_Pos_Vel(motor, safe_pos, velocity)
        state.last_gripper_msg = f"Moving to {percentage:.1f}% closed ({safe_pos:.3f} rad)"
        
    except Exception as e:
        state.last_gripper_msg = f"ERR: Gripper move failed: {e}"

# --- UI / Dashboard ---
def update_display():
    """Clears the screen and draws the combined dashboard."""
    # Clear screen using ANSI escape codes (works on most terminals including Windows)
    print('\033[2J\033[H', end='')
    
    # Force flush the output buffer
    import sys
    sys.stdout.flush()
    
    print("=" * 60)
    print("           COMPLETE GRIPPER CONTROL SYSTEM")
    print("=" * 60)
    
    # Stepper motor status
    print("ROTATION CONTROL (Stepper Motors):")
    print(f"  Target Position: M1={state.m1_target_pos:<4} | M2={state.m2_target_pos:<4}")
    print(f"  Speed (steps/s): {state.speed:<4}   | Step Size: {state.step_size:<4}")
    print(f"  Status: {'Connected' if state.stepper_connected else 'Disconnected'}")
    print(f"  Last Message: {state.last_stepper_msg}")
    
    print("-" * 60)
    
    # Gripper status
    gripper_status = "Connected" if state.gripper_connected else "Disconnected"
    if not DM_AVAILABLE:
        gripper_status = "Not Available"
    
    print("GRIPPER CONTROL (DM Motor):")
    print(f"  Closure: {state.gripper_closure_percent:.1f}% | Position: {state.gripper_position_rad:.3f} rad")
    print(f"  Status: {gripper_status}")
    print(f"  Last Message: {state.last_gripper_msg}")
    
    print("-" * 60)
    
    # Controls
    print("CONTROLS:")
    print("  Rotation:  [A/D] Rotate | [W/S] Speed | [Q/E] Step Size")
    print("  Gripper:   [O] Open | [C] Close")
    print("  General:   [SPACE] STOP | [R] Reset Pos | [ESC] Quit")
    print("=" * 60)

# --- Keyboard Input Handling ---
def on_press(key):
    """Handles key press events for both stepper and gripper control."""
    try:
        # --- Stepper Motor Controls ---
        if key.char == 'a':  # Rotate left
            new_m1 = state.m1_target_pos + state.step_size
            new_m2 = state.m2_target_pos - state.step_size
            send_stepper_move_command(stepper_ser, new_m1, new_m2, state.speed)
        elif key.char == 'd':  # Rotate right
            new_m1 = state.m1_target_pos - state.step_size
            new_m2 = state.m2_target_pos + state.step_size
            send_stepper_move_command(stepper_ser, new_m1, new_m2, state.speed)
        
        # --- Speed and Step Size ---
        elif key.char == 'w':
            state.speed += 50
        elif key.char == 's':
            state.speed = max(50, state.speed - 50)
        elif key.char == 'e':
            state.step_size += 10
        elif key.char == 'q':
            state.step_size = max(10, state.step_size - 10)

        # --- Gripper Controls ---
        elif key.char == 'o':  # Open gripper fully
            move_gripper_to_percentage(gripper_motor_control, gripper_motor, 0)
        elif key.char == 'c':  # Close gripper fully
            move_gripper_to_percentage(gripper_motor_control, gripper_motor, 100)

        # --- Utility Commands ---
        elif key.char == 'r':  # Reset stepper position to center
            send_stepper_move_command(stepper_ser, CONFIG["initial_pos"], CONFIG["initial_pos"], state.speed)

    except AttributeError:
        # Handle special keys like spacebar and escape
        if key == keyboard.Key.space:
            # Stop all motors
            send_stepper_command(stepper_ser, "<STOP>\n")
            state.last_stepper_msg = "STOP command sent"
            state.last_gripper_msg = "Manual stop requested"
        elif key == keyboard.Key.esc:
            # Stop the listener and the program
            state.running = False
            return False

# --- Main Execution ---
def main():
    global stepper_ser, gripper_motor, gripper_motor_control, gripper_serial_port
    
    stepper_ser = None
    gripper_motor = None
    gripper_motor_control = None
    gripper_serial_port = None
    
    try:
        print("Initializing Complete Gripper Control System...")
        
        # --- Setup Stepper Motor Connection ---
        try:
            stepper_ser = serial.Serial(CONFIG["stepper_port"], CONFIG["stepper_baud_rate"], timeout=1)
            state.stepper_connected = True
            state.last_stepper_msg = f"Connected to {CONFIG['stepper_port']}"
            print(f"✓ Stepper motor connected to {CONFIG['stepper_port']}")
            time.sleep(2)  # Give Arduino time to boot
        except serial.SerialException as e:
            print(f"❌ Could not connect to stepper motor on {CONFIG['stepper_port']}: {e}")
            state.last_stepper_msg = f"Connection failed: {e}"
        
        # --- Setup Gripper Motor Connection ---
        if DM_AVAILABLE:
            gripper_motor, gripper_motor_control, gripper_serial_port = setup_gripper()
            if gripper_motor:
                print("✓ Gripper motor connected and enabled")
            else:
                print("❌ Gripper motor setup failed")
        else:
            print("❌ DM motor library not available - gripper control disabled")



        # Start the background thread for reading stepper serial messages
        if stepper_ser and stepper_ser.is_open:
            reader = threading.Thread(target=stepper_reader, args=(stepper_ser, state))
            reader.daemon = True
            reader.start()

            # Perform initial homing of stepper motors
            print("Performing stepper motor homing...")
            send_stepper_command(stepper_ser, "<HOME>\n")
            time.sleep(10)  # Give homing time to complete

            # Move stepper to starting center position
            send_stepper_move_command(stepper_ser, CONFIG["initial_pos"], CONFIG["initial_pos"], state.speed)

        # Initialize gripper to open position
        if gripper_motor and gripper_motor_control:
            print("Initializing gripper to open position...")
            move_gripper_to_percentage(gripper_motor_control, gripper_motor, 0)
            time.sleep(2)

        print("\n" + "="*60)
        print("           SYSTEM READY - STARTING CONTROL LOOP")
        print("="*60)
        print()  # Add a blank line before the control loop starts
        
        # Small delay to ensure clean transition to control loop
        time.sleep(0.5)

        # Start the keyboard listener
        with keyboard.Listener(on_press=on_press) as listener:
            last_update = 0
            while state.running:
                current_time = time.time()
                # Only update display every 0.5 seconds to reduce flickering
                if current_time - last_update >= 0.5:
                    update_display()
                    last_update = current_time
                time.sleep(0.1)  # Faster polling for keyboard input
            listener.join()
    
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt received. Shutting down...")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        print("\nExiting program...")
        state.running = False
        
        # Release any pinched objects by opening gripper first
        if gripper_motor_control and gripper_motor:
            try:
                print("Opening gripper to release objects...")
                move_gripper_to_percentage(gripper_motor_control, gripper_motor, 0)  # Open to 0%
                time.sleep(1)  # Give time for gripper to open
                print("✓ Gripper opened")
            except:
                print("❌ Failed to open gripper")
        
        # Clean shutdown of stepper motor
        if stepper_ser and stepper_ser.is_open:
            try:
                print("Stopping stepper motors...")
                send_stepper_command(stepper_ser, "<STOP>\n")
                time.sleep(0.1)
                send_stepper_move_command(stepper_ser, 0, 0, 500)
                time.sleep(2)
                stepper_ser.close()
                print("✓ Stepper motor serial port closed")
            except:
                pass
        
        # Clean shutdown of gripper motor
        if gripper_motor_control and gripper_motor:
            try:
                print("Disabling gripper motor...")
                gripper_motor_control.disable(gripper_motor)
                print("✓ Gripper motor disabled")
            except:
                pass
        
        if gripper_serial_port and gripper_serial_port.is_open:
            try:
                gripper_serial_port.close()
                print("✓ Gripper serial port closed")
            except:
                pass
        

        
        print("✓ Cleanup completed")

if __name__ == "__main__":
    main()
