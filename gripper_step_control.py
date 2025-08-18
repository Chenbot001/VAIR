# File: gripper_complete.py
"""
Complete Gripper Control System
------------------------------
Integrated control for both stepper motor rotation and DM motor gripper actuation.
This script combines:
- Stepper motor control for gripper rotation (via Arduino)
- DM motor control for gripper open/close (via CAN)
- Depth sensor integration for object detection

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
import cv2
import numpy as np

from pynput import keyboard

# Add paths for local imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'damiao', 'DM_Control'))

# Import sensor functionality
try:
    from dmrobotics import Sensor, put_arrows_on_image
    SENSOR_AVAILABLE = True
except ImportError:
    print("Warning: Sensor library not available. Depth sensing disabled.")
    SENSOR_AVAILABLE = False

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
    "initial_speed": 50,
    
    # Gripper motor configuration
    "gripper_port": "COM3",
    "gripper_baud_rate": 921600,
    "gripper_motor_id": 0x01,
    "gripper_can_id": 0x11,
    
    # Sensor configuration
    "sensor_serial_id": 0,
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
        
        # Sensor state
        self.max_depth_intensity = 0.0
        self.baseline_intensity = 0.0  # Baseline intensity when gripper is fully open
        self.net_intensity = 0.0       # Net increase in intensity (current - baseline)
        self.sensor_connected = False
        self.last_sensor_msg = ""
        
        # Adaptive gripping state
        self.adaptive_gripping_enabled = True
        self.gripping_threshold = 0.5  # Lower threshold for more sensitive detection
        self.is_gripping = False       # Whether currently in gripping operation
        self.gripping_started = False  # Whether gripping operation has started
        self.gripping_thread = None    # Reference to the gripping thread
        
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

# --- Sensor Functions ---
def setup_sensor():
    """Initialize and configure the depth sensor"""
    if not SENSOR_AVAILABLE:
        return None
    
    try:
        sensor = Sensor(CONFIG["sensor_serial_id"])
        state.sensor_connected = True
        state.last_sensor_msg = "Sensor connected successfully"
        return sensor
    except Exception as e:
        state.last_sensor_msg = f"ERR: Sensor setup failed: {e}"
        return None

def get_max_depth_intensity(sensor):
    """
    Get the sensor depth image and return the maximum intensity as a floating point value.
    
    Args:
        sensor: The sensor object from dmrobotics
        
    Returns:
        float: Maximum depth intensity value, or 0.0 if sensor is not available
    """
    if not sensor or not SENSOR_AVAILABLE:
        return 0.0
    
    try:
        # Get the depth data from sensor
        depth = sensor.getDepth()
        
        if depth is not None and depth.size > 0:
            # Find the maximum intensity value in the depth image
            max_intensity = float(np.max(depth))
            state.max_depth_intensity = max_intensity
            
            # Calculate net intensity (current - baseline)
            state.net_intensity = max_intensity - state.baseline_intensity
            
            state.last_sensor_msg = f"Max: {max_intensity:.3f}, Net: {state.net_intensity:.3f}"
            return max_intensity
        else:
            state.last_sensor_msg = "No depth data available"
            return 0.0
            
    except Exception as e:
        state.last_sensor_msg = f"ERR: Failed to get depth intensity: {e}"
        return 0.0

def calibrate_baseline_intensity(sensor, samples=10):
    """
    Calibrate the baseline intensity when gripper is fully open.
    
    Args:
        sensor: The sensor object from dmrobotics
        samples: Number of samples to average for baseline
    """
    if not sensor or not SENSOR_AVAILABLE:
        state.last_sensor_msg = "ERR: Sensor not available for calibration"
        return False
    
    try:
        print("Calibrating baseline intensity...")
        total_intensity = 0.0
        valid_samples = 0
        
        for i in range(samples):
            intensity = get_max_depth_intensity(sensor)
            if intensity > 0.0:
                total_intensity += intensity
                valid_samples += 1
            time.sleep(0.1)
        
        if valid_samples > 0:
            state.baseline_intensity = total_intensity / valid_samples
            state.net_intensity = 0.0  # Reset net intensity
            state.last_sensor_msg = f"Baseline calibrated: {state.baseline_intensity:.3f}"
            print(f"✓ Baseline intensity calibrated: {state.baseline_intensity:.3f}")
            return True
        else:
            state.last_sensor_msg = "ERR: No valid samples for calibration"
            return False
            
    except Exception as e:
        state.last_sensor_msg = f"ERR: Calibration failed: {e}"
        return False

def start_adaptive_gripping():
    """Start an adaptive gripping operation"""
    state.is_gripping = True
    state.gripping_started = True
    state.last_gripper_msg = "Starting adaptive gripping..."

def stop_adaptive_gripping():
    """Stop the current adaptive gripping operation"""
    state.is_gripping = False
    state.gripping_started = False
    state.gripping_thread = None
    state.last_gripper_msg = "Adaptive gripping stopped"

def check_gripping_condition():
    """
    Check if the net intensity exceeds the threshold during gripping.
    Returns True if object detected (should stop gripping).
    Based on the working trigger_teleop_fb.py approach.
    """
    if not state.is_gripping:
        return False
    
    # Use the same threshold logic as the working script
    return state.net_intensity > state.gripping_threshold

def handle_adaptive_gripping(motor_control, motor, target_percentage, velocity=0.3):
    """
    Handle adaptive gripping by continuously monitoring sensor and stopping when object detected.
    Based on the working trigger_teleop_fb.py approach.
    """
    if not motor_control or not motor:
        state.last_gripper_msg = "ERR: Motor not available for adaptive gripping"
        return
    
    # Ensure sensor is connected and providing data
    if not state.sensor_connected or state.max_depth_intensity == 0.0:
        state.last_gripper_msg = "ERR: Sensor not ready for adaptive gripping"
        return
    
    # Ensure we're not already gripping
    if state.is_gripping:
        state.last_gripper_msg = "ERR: Already in gripping operation"
        return
    
    try:
        # Start adaptive gripping state
        start_adaptive_gripping()
        state.last_gripper_msg = "Starting adaptive gripping..."
        
        # Get current position to use as hold position if needed
        try:
            current_pos = motor.getPosition()
        except:
            # Fallback to current state position if getPosition() fails
            current_pos = state.gripper_position_rad
        
        # Send command to move toward fully closed
        target_position = percentage_to_position(target_percentage)
        safe_pos = safe_gripper_position(target_position)
        motor_control.control_Pos_Vel(motor, safe_pos, velocity)
        
        # Update state
        state.gripper_closure_percent = target_percentage
        state.gripper_position_rad = safe_pos
        
        # Monitor sensor while gripper is moving
        while state.is_gripping:
            # Check sensor feedback
            if check_gripping_condition():
                # Object detected - get current position and hold it
                try:
                    hold_pos = motor.getPosition()
                except:
                    # Fallback to current state position
                    hold_pos = state.gripper_position_rad
                
                # Send command to hold current position with zero velocity to stop movement
                motor_control.control_Pos_Vel(motor, hold_pos, 0.0)
                
                # Send multiple stop commands to ensure motor stops
                for _ in range(3):
                    motor_control.control_Pos_Vel(motor, hold_pos, 0.0)
                    time.sleep(0.01)
                
                current_percentage = position_to_percentage(hold_pos)
                print(f"\n[STOP] Object detected! Holding at {current_percentage:.1f}% closed")
                # Update state to reflect the actual holding position
                state.gripper_position_rad = hold_pos
                state.gripper_closure_percent = current_percentage
                state.last_gripper_msg = f"Object detected! Holding at {current_percentage:.1f}% closed"
                stop_adaptive_gripping()
                break
            else:
                # No object detected - continue monitoring
                # Get current position for display
                try:
                    current_pos = motor.getPosition()
                except:
                    current_pos = state.gripper_position_rad
                
                # Debug output
                current_percentage = position_to_percentage(current_pos)
                print(f"\r[GRIP] {current_percentage:.1f}% | Net: {state.net_intensity:.3f} | Threshold: {state.gripping_threshold:.3f}", end="")
                
                # Much faster polling for better responsiveness
                time.sleep(0.01)  # 100Hz polling
                
                # Check if we've reached the target without object detection
                if current_percentage >= target_percentage:
                    state.last_gripper_msg = f"Reached target {target_percentage:.1f}% without object detection"
                    stop_adaptive_gripping()
                    break
            
    except Exception as e:
        state.last_gripper_msg = f"ERR: Adaptive gripping failed: {e}"
        stop_adaptive_gripping()


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

def sensor_reader(sensor, state):
    """Continuously reads from the sensor and updates depth intensity in the background."""
    while state.running:
        try:
            if sensor and state.sensor_connected:
                get_max_depth_intensity(sensor)
                # Read more frequently when gripping is active
                if state.is_gripping:
                    time.sleep(0.01)  # 100Hz when gripping for maximum responsiveness
                else:
                    time.sleep(0.1)   # 10Hz when not gripping
        except Exception as e:
            state.last_sensor_msg = f"ERR: Sensor reading failed: {e}"
            state.sensor_connected = False
            break

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

def move_gripper_to_percentage(motor_control, motor, percentage, velocity=0.3):
    """
    Move gripper to a specific closure percentage with adaptive gripping for closing operations.
    
    Args:
        motor_control: The motor control object
        motor: The motor object
        percentage: Target closure percentage (0-100)
        velocity: Movement velocity
    """
    if not motor_control or not motor:
        state.last_gripper_msg = "ERR: Gripper not available"
        return
    
    # Stop any existing adaptive gripping operation
    if state.is_gripping:
        stop_adaptive_gripping()
        time.sleep(0.1)  # Brief pause to ensure clean state
    
    try:
        # Convert percentage to radians
        target_position = percentage_to_position(percentage)
        safe_pos = safe_gripper_position(target_position)
        
        # Update state
        state.gripper_closure_percent = percentage
        state.gripper_position_rad = safe_pos
        
        if percentage > 0:
            # Always use adaptive gripping for closing operations
            state.last_gripper_msg = f"Starting adaptive grip to {percentage:.1f}%"
            # Start adaptive gripping in a separate thread with slower velocity
            import threading
            grip_thread = threading.Thread(
                target=handle_adaptive_gripping, 
                args=(motor_control, motor, percentage, 0.3)  # Use 0.3 rad/s for adaptive gripping
            )
            grip_thread.daemon = True
            state.gripping_thread = grip_thread  # Store reference to thread
            grip_thread.start()
        else:
            # Direct movement for opening
            motor_control.control_Pos_Vel(motor, safe_pos, 0.3)  # Use 0.3 rad/s for opening too
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
    
    # Sensor status
    sensor_status = "Connected" if state.sensor_connected else "Disconnected"
    if not SENSOR_AVAILABLE:
        sensor_status = "Not Available"
    
    print("DEPTH SENSOR:")
    print(f"  Max Intensity: {state.max_depth_intensity:.3f}")
    print(f"  Baseline: {state.baseline_intensity:.3f} | Net: {state.net_intensity:.3f}")
    print(f"  Status: {sensor_status}")
    
    print("-" * 60)
    
    # Adaptive gripping status
    gripping_status = "Active" if state.is_gripping else "Inactive"
    
    print("ADAPTIVE GRIPPING:")
    print(f"  Threshold: {state.gripping_threshold:.3f}")
    if state.is_gripping:
        print(f"  Net Intensity: {state.net_intensity:.3f} / {state.gripping_threshold:.3f}")
        print(f"  Will Stop: {state.net_intensity > state.gripping_threshold}")
    
    print("-" * 60)
    
    # Controls
    print("CONTROLS:")
    print("  Rotation:  [A/D] Rotate | [W/S] Speed | [Q/E] Step Size")
    print("  Gripper:   [O] Open | [C] Close (Adaptive)")
    print("  Sensor:    [B] Calibrate Baseline")
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
        elif key.char == 'c':  # Close gripper (adaptive)
            # Add a small delay to prevent double-press issues
            if not state.is_gripping:
                move_gripper_to_percentage(gripper_motor_control, gripper_motor, 100)
            else:
                state.last_gripper_msg = "Gripping already in progress"

        # --- Sensor Controls ---
        elif key.char == 'b':  # Calibrate baseline intensity
            if sensor:
                calibrate_baseline_intensity(sensor)
            else:
                state.last_sensor_msg = "ERR: Sensor not connected"

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
    global stepper_ser, gripper_motor, gripper_motor_control, gripper_serial_port, sensor
    
    stepper_ser = None
    gripper_motor = None
    gripper_motor_control = None
    gripper_serial_port = None
    sensor = None
    
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

        # --- Setup Sensor Connection ---
        if SENSOR_AVAILABLE:
            sensor = setup_sensor()
            if sensor:
                print("✓ Depth sensor connected")
                # Calibrate baseline intensity when sensor is first initialized
                print("Calibrating baseline intensity...")
                if calibrate_baseline_intensity(sensor):
                    print("✓ Baseline intensity calibrated")
                else:
                    print("⚠ Baseline calibration failed")
            else:
                print("❌ Depth sensor setup failed")
        else:
            print("❌ Sensor library not available - depth sensing disabled")


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

        # Start the background thread for reading sensor data
        if sensor and state.sensor_connected:
            sensor_reader_thread = threading.Thread(target=sensor_reader, args=(sensor, state))
            sensor_reader_thread.daemon = True
            sensor_reader_thread.start()
            print("✓ Sensor reader thread started")
            
            # Give sensor reader time to start and get initial readings
            print("Waiting for sensor to stabilize...")
            time.sleep(1.0)  # Allow sensor reader to start and get initial data

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
        
        # Clean shutdown of sensor
        if sensor and state.sensor_connected:
            try:
                print("Disconnecting sensor...")
                sensor.disconnect()
                print("✓ Sensor disconnected")
            except:
                pass
        

        
        print("✓ Cleanup completed")

if __name__ == "__main__":
    main()
