# File: gripper_angle_control_encoder.py
"""
Advanced Angle-Based Gripper Control System with Encoder Feedback
----------------------------------------------------------------
Integrated control system combining stepper motor rotation, DM motor gripper actuation,
rotary encoder feedback, and depth sensor integration for precise robotic manipulation.

This script provides:
- Stepper motor control for gripper rotation (via Arduino) with angle-based positioning
- DM motor control for gripper open/close (via CAN) with adaptive gripping
- Rotary encoder integration for real-time angle measurement and validation
- Depth sensor integration for object detection with visual feedback
- Object diameter-based calibration for precise angle control
- Automatic encoder data recording during rotation operations
- Real-time sensor image display (depth, deformation, shear)

Stepper Motor Controls:
- [A/D] Rotate gripper CCW/CW by target angle (triggers encoder recording)
- [W/S] Adjust rotation speed
- [Q/E] Adjust target angle in 5° increments
- [2/3/4/5] Set object diameter (2-5mm) for calibration

Gripper Controls:
- [O] Open gripper fully
- [C] Close gripper with adaptive gripping (sensor-based object detection)

Sensor Controls:
- [B] Calibrate baseline intensity for adaptive gripping

Encoder Controls:
- [Z] Zero rotary encoder position

General Controls:
- [SPACE] STOP all motors
- [X] Reset stepper position to center and zero angle tracking
- [ESC] Quit program

Features:
- Automatic 5-second encoder recording during rotation commands
- Adaptive gripping stops when object detected via depth sensor
- Real-time sensor visualization with depth, deformation, and shear data
- Diameter-based angle calibration for different object sizes
- Comprehensive live dashboard with all system status
"""

import serial
import time
import os
import threading
import sys
import math
import cv2
import numpy as np
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import minimalmodbus
import signal


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

# --- Encoder Calibration Data ---
# Steps per degree values from regression analysis for different diameters and directions
ENCODER_CALIBRATION = {
    # Clockwise direction
    'cw': {
        2: 0.652042,  # steps per degree
        3: 0.988245,
        4: 1.276730,
        5: 1.643381
    },
    # Counter-clockwise direction  
    'ccw': {
        2: 0.653365,  # steps per degree
        3: 0.994351,
        4: 1.292792,
        5: 1.638945
    }
}

# --- Configuration ---
CONFIG = {
    # Stepper motor configuration
    "stepper_port": "COM9",
    "stepper_baud_rate": 115200,
    "microsteps": 16,
    "max_steps": 1000,
    "initial_pos": 500,
    "initial_speed": 50,
    "homing_speed": 500,  # Fast speed for homing and initialization operations
    
    # Angle-based control configuration
    "min_diameter_mm": 2,
    "max_diameter_mm": 5,
    "initial_diameter_mm": 2,
    "angle_increment_deg": 5,
    "initial_target_angle_deg": 90,  # Default target angle setting
    "max_angle_deg": 180,  # Maximum rotation angle from center
    
    # Gripper motor configuration
    "gripper_port": "COM3",
    "gripper_baud_rate": 921600,
    "gripper_motor_id": 0x01,
    "gripper_can_id": 0x11,
    
    # Sensor configuration
    "sensor_serial_id": 0,
    
    # Encoder configuration
    "encoder_port": "COM11",
    "encoder_slave_address": 1,
    "encoder_baudrate": 9600,
    "encoder_read_register": 0,
    "encoder_zero_register": 8,
    "encoder_direction_register": 9,
}

# Gripper position limits (in radians)
GRIPPER_MIN_POS = -1.38  # Fully closed position
GRIPPER_MAX_POS = 0.0    # Fully open position

# Constants for encoder decoding
SINGLE_TURN_RESOLUTION = 2**15

# --- State Tracking ---
class SystemState:
    """A class to hold the live state of both motor controllers and sensor."""
    def __init__(self):
        # Stepper motor state (step-based)
        self.m1_target_pos = CONFIG["initial_pos"]
        self.m2_target_pos = CONFIG["initial_pos"]
        self.speed = CONFIG["initial_speed"]
        
        # Angle-based control state
        self.current_angle_deg = 0.0  # Current angle from center position
        self.target_angle_deg = CONFIG["initial_target_angle_deg"]  # Target angle increment
        self.object_diameter_mm = CONFIG["initial_diameter_mm"]  # Current object diameter
        
        # Gripper state
        self.gripper_closure_percent = 0  # 0% = open, 100% = closed
        self.gripper_position_rad = GRIPPER_MAX_POS
        
        # Sensor state
        self.max_depth_intensity = 0.0
        self.baseline_intensity = 0.0  # Baseline intensity when gripper is fully open
        self.net_intensity = 0.0       # Net increase in intensity (current - baseline)
        self.sensor_connected = False
        self.last_sensor_msg = ""
        
        # Sensor image display state
        self.display_sensor_images = True  # Always display sensor images when available
        self.sensor_fps = 0.0  # Sensor reading FPS
        self.frame_count = 0  # Frame counter for FPS calculation
        self.fps_start_time = 0.0  # Start time for FPS calculation
        
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
        
        # Encoder state
        self.encoder_connected = False
        self.last_encoder_msg = ""
        self.is_ccw = False  # Start with default direction CW
        self.is_recording = False
        self.recorded_data = []
        self.recording_start_time = None
        # Recording metadata for CSV
        self.recording_diameter = 0
        self.recording_target_angle = 0.0
        self.recording_grip_strength = 0.0
        self.recording_initial_angle = 0.0
        self.recording_direction = 'cw'

# Global state object
state = SystemState()

# Global variables for cleanup
listener = None

def signal_handler(signum, frame):
    """Handle interrupt signals for graceful shutdown"""
    print(f"\n\nReceived signal {signum}. Shutting down gracefully...")
    state.running = False

# --- Angle Conversion Functions ---
def get_steps_per_degree(diameter_mm, direction):
    """
    Get steps per degree for given diameter and direction from calibration data.
    Uses interpolation for diameters between calibrated values.
    
    Args:
        diameter_mm: Object diameter in millimeters (1-5)
        direction: 'cw' or 'ccw'
        
    Returns:
        float: Steps per degree
    """
    diameter_mm = max(CONFIG["min_diameter_mm"], min(CONFIG["max_diameter_mm"], diameter_mm))
    
    calibration = ENCODER_CALIBRATION[direction]
    
    # If exact diameter exists, return it
    if diameter_mm in calibration:
        return calibration[diameter_mm]
    
    # Otherwise, interpolate between nearest values
    diameters = sorted(calibration.keys())
    
    # Find the two closest diameters
    lower_d = max([d for d in diameters if d <= diameter_mm])
    upper_d = min([d for d in diameters if d >= diameter_mm])
    
    if lower_d == upper_d:
        return calibration[lower_d]
    
    # Linear interpolation
    lower_steps = calibration[lower_d]
    upper_steps = calibration[upper_d]
    
    ratio = (diameter_mm - lower_d) / (upper_d - lower_d)
    interpolated_steps = lower_steps + ratio * (upper_steps - lower_steps)
    
    return interpolated_steps

def angle_to_steps(angle_deg, diameter_mm, direction):
    """
    Convert angle in degrees to motor steps for given diameter and direction.
    
    Args:
        angle_deg: Angle in degrees
        diameter_mm: Object diameter in millimeters
        direction: 'cw' or 'ccw'
        
    Returns:
        float: Number of steps (will be rounded to 1/16 microstep)
    """
    steps_per_deg = get_steps_per_degree(diameter_mm, direction)
    total_steps = angle_deg * steps_per_deg
    
    # Round to nearest 1/16 microstep
    microstep_resolution = 1.0 / CONFIG["microsteps"]
    rounded_steps = round(total_steps / microstep_resolution) * microstep_resolution
    
    return rounded_steps

def steps_to_angle(steps, diameter_mm, direction):
    """
    Convert motor steps to angle in degrees for given diameter and direction.
    
    Args:
        steps: Number of steps
        diameter_mm: Object diameter in millimeters
        direction: 'cw' or 'ccw'
        
    Returns:
        float: Angle in degrees
    """
    steps_per_deg = get_steps_per_degree(diameter_mm, direction)
    if steps_per_deg == 0:
        return 0.0
    return steps / steps_per_deg

def calculate_motor_positions(target_angle_deg, direction):
    """
    Calculate motor positions for a given target angle and direction.
    
    Args:
        target_angle_deg: Target angle in degrees
        direction: 'cw' or 'ccw'
        
    Returns:
        tuple: (m1_pos, m2_pos) - new motor positions
    """
    # Get the step increment for this angle and direction
    step_increment = angle_to_steps(abs(target_angle_deg), state.object_diameter_mm, direction)
    
    # Calculate new positions based on direction
    if direction == 'ccw':  # A key - counter-clockwise
        new_m1 = state.m1_target_pos + step_increment
        new_m2 = state.m2_target_pos - step_increment
    else:  # direction == 'cw' - D key - clockwise
        new_m1 = state.m1_target_pos - step_increment
        new_m2 = state.m2_target_pos + step_increment
    
    # Clamp to physical limits
    new_m1 = max(0, min(CONFIG["max_steps"], new_m1))
    new_m2 = max(0, min(CONFIG["max_steps"], new_m2))
    
    return new_m1, new_m2

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
        # print("Calibrating baseline intensity...")
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

def display_sensor_images(sensor):
    """
    Display concatenated sensor images including depth, deformation, and shear in a single window.
    Based on the implementation from main.py.
    
    Args:
        sensor: The sensor object from dmrobotics
    """
    if not sensor or not SENSOR_AVAILABLE or not state.display_sensor_images:
        return
    
    try:
        # Get sensor data
        img = sensor.getRawImage()
        depth = sensor.getDepth()  # output the deformed depth
        deformation = sensor.getDeformation2D()
        shear = sensor.getShear()
        
        # Create depth image with color mapping
        depth_img = cv2.applyColorMap((depth * 0.25 * 255.0).astype('uint8'), cv2.COLORMAP_HOT)
        
        # Create black background for deformation and shear arrows
        black_img = np.zeros_like(img)
        black_img = np.stack([black_img] * 3, axis=-1)
        
        # Create deformation and shear images
        deformation_img = put_arrows_on_image(black_img.copy(), deformation * 20)
        shear_img = put_arrows_on_image(black_img.copy(), shear * 20)
        
        # Ensure all images have the same height for concatenation
        height = depth_img.shape[0]
        width = depth_img.shape[1]
        
        # Resize deformation and shear images to match depth image dimensions if needed
        if deformation_img.shape[:2] != (height, width):
            deformation_img = cv2.resize(deformation_img, (width, height))
        if shear_img.shape[:2] != (height, width):
            shear_img = cv2.resize(shear_img, (width, height))
        
        # Add text labels to each image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_color = (255, 255, 255)
        thickness = 2
        
        # Add labels
        cv2.putText(depth_img, 'Depth', (10, 30), font, font_scale, font_color, thickness)
        cv2.putText(deformation_img, 'Deformation', (10, 30), font, font_scale, font_color, thickness)
        cv2.putText(shear_img, 'Shear', (10, 30), font, font_scale, font_color, thickness)
        
        # Concatenate images horizontally
        combined_img = np.hstack([depth_img, deformation_img, shear_img])
        
        # Display the combined image
        cv2.imshow('Sensor Data - Depth | Deformation | Shear', combined_img)
        
        # Update FPS calculation
        state.frame_count += 1
        current_time = time.time()
        if current_time - state.fps_start_time > 1.0:
            state.sensor_fps = state.frame_count / (current_time - state.fps_start_time)
            state.frame_count = 0
            state.fps_start_time = current_time
        
        # Check for window close events (non-blocking)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            # Reset sensor when 'r' is pressed in the sensor window
            sensor.reset()
            state.last_sensor_msg = "Sensor reset via image window"
        
    except Exception as e:
        state.last_sensor_msg = f"ERR: Image display failed: {e}"

def close_sensor_windows():
    """Close the sensor image window"""
    try:
        cv2.destroyWindow('Sensor Data - Depth | Deformation | Shear')
    except:
        pass

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

def handle_adaptive_gripping(motor_control, motor, target_percentage, velocity=0.1):
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


# --- Encoder Functions ---
def unwrap_angles(angles):
    """
    Unwrap angle data to handle 0°/360° transitions smoothly.
    
    Args:
        angles (list): List of angle values in degrees
    
    Returns:
        list: Unwrapped angle values (continuous, may exceed 360°)
    """
    if not angles:
        return angles
    
    unwrapped = [angles[0]]  # Start with the first angle
    
    for i in range(1, len(angles)):
        prev_angle = unwrapped[i-1]
        current_angle = angles[i]
        
        # Calculate the difference
        diff = current_angle - (prev_angle % 360)
        
        # Handle wrap-around cases
        if diff > 180:  # Wrapped from ~360° to ~0°
            adjustment = -360
        elif diff < -180:  # Wrapped from ~0° to ~360°
            adjustment = 360
        else:
            adjustment = 0
        
        # Add the adjusted angle
        unwrapped.append(prev_angle + diff + adjustment)
    
    return unwrapped

def zero_encoder(instrument):
    """Sends the command to zero the encoder."""
    try:
        print("\nSending zeroing command...")
        instrument.write_register(
            registeraddress=CONFIG["encoder_zero_register"],
            value=1,
            functioncode=6
        )
        print("Zeroing command sent successfully!")
    except minimalmodbus.ModbusException as e:
        print(f"Failed to send zeroing command: {e}")

def set_encoder_direction(instrument, set_to_ccw):
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
            registeraddress=CONFIG["encoder_direction_register"],
            value=value_to_write,
            functioncode=6
        )
        print("Direction set successfully!")
        return set_to_ccw
    except minimalmodbus.ModbusException as e:
        print(f"Failed to set direction: {e}")
        # If the command fails, return the previous state
        return not set_to_ccw

def start_encoder_recording(direction=None):
    """Start recording encoder data."""
    global state
    
    # Check if gripper is closed (gripper_closure_percent > 0)
    if state.gripper_closure_percent <= 0:
        print(f"⚠ Recording disabled: Gripper is open ({state.gripper_closure_percent:.1f}% closed)")
        return
    
    if not state.is_recording:
        state.is_recording = True
        state.recorded_data = []
        state.recording_start_time = time.time()
        # Store initial conditions for CSV data
        state.recording_diameter = state.object_diameter_mm
        state.recording_target_angle = state.target_angle_deg
        state.recording_grip_strength = state.max_depth_intensity
        state.recording_direction = direction if direction is not None else ('ccw' if state.is_ccw else 'cw')
        # Get initial angle from encoder
        if encoder_instrument and state.encoder_connected:
            initial_angle, _, _, _, _ = read_encoder_data(encoder_instrument)
            state.recording_initial_angle = initial_angle if initial_angle is not None else 0.0
        else:
            state.recording_initial_angle = 0.0
        print(f"\n🎙️  Encoder recording started at {datetime.now().strftime('%H:%M:%S')}")

def stop_encoder_recording():
    """Stop recording encoder data and save to file."""
    global state
    if state.is_recording:
        state.is_recording = False
        recording_duration = time.time() - state.recording_start_time
        print(f"\n⏹️  Encoder recording stopped. Duration: {recording_duration:.2f}s")
        print(f"📊 Collected {len(state.recorded_data)} data points")
        
        if state.recorded_data:
            try:
                save_and_plot_encoder_data(state.recorded_data, recording_duration)
            except Exception as e:
                print(f"❌ Error during data processing: {e}")
        else:
            print("⚠ No data collected during recording")
        
        state.recorded_data = []
        state.recording_start_time = None

def save_and_plot_encoder_data(data, duration):
    """Save encoder data to CSV and create plots."""
    if not data:
        print("No encoder data to save/plot")
        return
    
    # Create filename with new naming convention including direction
    diameter = state.recording_diameter
    target_angle = state.recording_target_angle
    direction = state.recording_direction
    plot_filename = f"rotary_encoder/plot/plot_{diameter}_{target_angle}_{direction}.png"
    csv_filename = f"rotary_encoder/encoder_data.csv"
    
    # Ensure directories exist
    os.makedirs("rotary_encoder", exist_ok=True)
    os.makedirs("rotary_encoder/plot", exist_ok=True)
    
    # Extract data for low velocity calculation
    times = [row[1] for row in data]
    angles = [row[2] for row in data]
    
    # Unwrap angles to handle 0°/360° transitions
    unwrapped_angles = unwrap_angles(angles)
    
    # Calculate gradient (angular velocity) using unwrapped angles
    gradients = []
    gradient_times = []
    
    for i in range(1, len(unwrapped_angles)):
        dt = times[i] - times[i-1]
        dangle = unwrapped_angles[i] - unwrapped_angles[i-1]
        if dt > 0:  # Avoid division by zero
            gradient = dangle / dt  # degrees per second
            gradients.append(gradient)
            gradient_times.append(times[i])
    
    # Find the first point where angular velocity drops below 1 deg/s
    low_velocity_point = None
    low_velocity_angle = None
    if gradients:
        for i, grad in enumerate(gradients):
            if abs(grad) < 1.0:  # First point below 1 deg/s
                low_velocity_point = gradient_times[i]
                # Find corresponding angle at this time
                for j, t in enumerate(times):
                    if t >= low_velocity_point:
                        low_velocity_angle = angles[j]
                        break
                break
    
    # Use the low velocity angle as measured angle, or fallback to last reading
    measured_angle = low_velocity_angle if low_velocity_angle is not None else (data[-1][2] if data else 0.0)
    
    # For CCW rotation, calculate the positive angle change from start to finish
    if state.recording_direction == 'ccw':
        # Calculate the angle change from initial to measured
        angle_change = measured_angle - state.recording_initial_angle
        
        # Handle wrap-around cases (0°/360° boundary)
        if angle_change > 180:  # Wrapped from ~360° to ~0°
            angle_change -= 360
        elif angle_change < -180:  # Wrapped from ~0° to ~360°
            angle_change += 360
        
        # For CCW, we want positive angle change, so negate if negative
        if angle_change < 0:
            angle_change = -angle_change
        
        # Use the calculated angle change as the measured angle for CCW
        measured_angle = angle_change
    
    # Calculate error
    error = measured_angle - target_angle
    
    # Check if error is within acceptable range (±10 degrees)
    if abs(error) <= 10.0:
        # Save to CSV with the specified format
        try:
            # Check if CSV file exists to determine if we need to write headers
            file_exists = os.path.exists(csv_filename)
            
            with open(csv_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Calculate steps using calibration data
                steps_per_degree = get_steps_per_degree(state.recording_diameter, state.recording_direction)
                calculated_steps = state.recording_target_angle * steps_per_degree
                # Round to nearest 1/16 microstep (1/16 = 0.0625)
                calculated_steps = round(calculated_steps / 0.0625) * 0.0625
                
                # Write headers if file doesn't exist
                if not file_exists:
                    writer.writerow(['diameter', 'grip_strength', 'direction', 'steps', 
                                    'initial_angle', 'target_angle', 'measured_angle', 'error'])
                
                # Write data row with rounded values
                writer.writerow([
                    state.recording_diameter,
                    round(state.recording_grip_strength, 4),
                    state.recording_direction,
                    round(calculated_steps, 4),
                    round(state.recording_initial_angle, 2),
                    round(state.recording_target_angle, 2),
                    round(measured_angle, 2),
                    round(error, 2)
                ])
            print(f"💾 Data point saved to CSV: diameter={diameter}mm, target={target_angle}°, measured={measured_angle:.1f}°, error={error:.1f}°")
        except Exception as e:
            print(f"❌ Error saving CSV: {e}")
    else:
        print(f"⚠ Data discarded: error {error:.1f}° exceeds ±10° threshold")
        return
    
    # Create plots
    try:
        # Use the already calculated data for plotting
        # (times, angles, gradients, gradient_times, low_velocity_point, low_velocity_angle are already available)
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        fig.suptitle(f'Rotary Encoder Data - {duration:.2f}s Recording', fontsize=16)
        
        # Plot 1: Original angle over time (0-360°)
        ax1.plot(times, angles, 'b-', linewidth=1.5)
        ax1.set_ylabel('Angle (degrees)')
        ax1.set_title('Angle vs Time')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Gradient (angular velocity) over time (using unwrapped angles)
        if gradients:
            ax2.plot(gradient_times, gradients, 'r-', linewidth=1.5)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angular Velocity (deg/s)')
            ax2.set_title('Angular Velocity vs Time (Smooth)')
            ax2.grid(True, alpha=0.3)
            ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)  # Zero line
            
            # Add threshold line at 1 deg/s
            ax2.axhline(y=1.0, color='g', linestyle='--', alpha=0.7, label='1 deg/s threshold')
            ax2.axhline(y=-1.0, color='g', linestyle='--', alpha=0.7)
            
            # Label the low velocity point if found
            if low_velocity_point is not None and low_velocity_angle is not None:
                # Find the gradient value at this point
                low_velocity_grad = None
                for i, t in enumerate(gradient_times):
                    if t >= low_velocity_point:
                        low_velocity_grad = gradients[i]
                        break
                
                if low_velocity_grad is not None:
                    # Add annotation to velocity plot
                    ax2.annotate(f'Angle: {low_velocity_angle:.1f}°\nVel: {low_velocity_grad:.1f}°/s', 
                                xy=(low_velocity_point, low_velocity_grad),
                                xytext=(low_velocity_point + 0.5, low_velocity_grad + 5),
                                arrowprops=dict(arrowstyle='->', color='red'),
                                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
                                fontsize=10)
                    
                    # Add vertical line to angle plot
                    ax1.axvline(x=low_velocity_point, color='red', linestyle='--', alpha=0.7)
                    ax1.annotate(f'Low Vel Point\nAngle: {low_velocity_angle:.1f}°', 
                                xy=(low_velocity_point, low_velocity_angle),
                                xytext=(low_velocity_point + 0.5, low_velocity_angle + 30),
                                arrowprops=dict(arrowstyle='->', color='red'),
                                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
                                fontsize=10)
        
        plt.tight_layout()
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"📈 Encoder plot saved to: {plot_filename}")
        
        # Close the plot to prevent window from opening
        plt.close(fig)
        
    except Exception as e:
        print(f"❌ Error creating plot: {e}")

def read_encoder_data(instrument):
    """Read and decode encoder data."""
    try:
        total_encoded_value = instrument.read_long(CONFIG["encoder_read_register"], 3, False)
        single_turn_value = total_encoded_value & 0x7FFF
        turn_count = total_encoded_value >> 15
        angle = (single_turn_value / SINGLE_TURN_RESOLUTION) * 360.0
        
        direction_str = "CCW" if state.is_ccw else "CW"
        current_time = time.time()
        
        # Record data if recording is active
        if state.is_recording and state.recording_start_time is not None:
            elapsed_time = current_time - state.recording_start_time
            state.recorded_data.append([
                datetime.now().strftime('%H:%M:%S.%f')[:-3],
                elapsed_time,
                angle,
                turn_count,
                single_turn_value,
                total_encoded_value,
                direction_str
            ])
        
        return angle, turn_count, single_turn_value, total_encoded_value, direction_str
        
    except minimalmodbus.ModbusException as e:
        state.last_encoder_msg = f"Modbus error: {e}"
        return None, None, None, None, None
    except Exception as e:
        state.last_encoder_msg = f"Encoder error: {e}"
        return None, None, None, None, None

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
    """Continuously reads from the sensor, updates depth intensity, and displays images in the background."""
    # Initialize FPS tracking
    state.fps_start_time = time.time()
    state.frame_count = 0
    
    while state.running:
        try:
            if sensor and state.sensor_connected:
                # Get depth intensity for gripping logic
                get_max_depth_intensity(sensor)
                
                # Display sensor images if enabled
                display_sensor_images(sensor)
                
                # Read more frequently when gripping is active
                if state.is_gripping:
                    time.sleep(0.01)  # 100Hz when gripping for maximum responsiveness
                else:
                    time.sleep(0.05)   # 20Hz when not gripping (increased from 10Hz for better image display)
        except Exception as e:
            state.last_sensor_msg = f"ERR: Sensor reading failed: {e}"
            state.sensor_connected = False
            break
    
    # Clean up image windows when exiting
    close_sensor_windows()

def encoder_reader(instrument, state):
    """Continuously reads from the encoder in the background."""
    while state.running:
        try:
            if instrument and state.encoder_connected:
                # Read encoder data
                angle, turn_count, single_turn, raw_value, direction = read_encoder_data(instrument)
                if angle is not None:
                    state.last_encoder_msg = f"Angle: {angle:.2f}° | Turns: {turn_count} | Dir: {direction}"
                
                # Check if recording should stop (5 seconds)
                if state.is_recording and state.recording_start_time is not None:
                    elapsed_time = time.time() - state.recording_start_time
                    if elapsed_time >= 5.0:  # 5 seconds recording
                        stop_encoder_recording()
                
                time.sleep(0.05)  # 20Hz reading rate
        except Exception as e:
            state.last_encoder_msg = f"ERR: Encoder reading failed: {e}"
            state.encoder_connected = False
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

def send_angle_move_command(ser, angle_deg, direction):
    """
    Send a move command based on angle and direction.
    
    Args:
        ser: Serial connection to Arduino
        angle_deg: Target angle increment in degrees
        direction: 'cw' or 'ccw'
    """
    if not ser or not ser.is_open:
        return
    
    # Calculate new motor positions
    new_m1, new_m2 = calculate_motor_positions(angle_deg, direction)
    
    # Send the move command
    send_stepper_move_command(ser, new_m1, new_m2, state.speed)
    
    # Update current angle based on actual movement
    if direction == 'ccw':
        state.current_angle_deg -= angle_deg  # CCW decreases angle (negative direction)
    else:  # cw
        state.current_angle_deg += angle_deg  # CW increases angle (positive direction)
    
    # Keep angle within reasonable bounds for display
    if state.current_angle_deg > 360:
        state.current_angle_deg -= 360
    elif state.current_angle_deg < -360:
        state.current_angle_deg += 360

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

def move_gripper_to_percentage(motor_control, motor, percentage, velocity=0.1):
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
                args=(motor_control, motor, percentage, 0.1)  # Use 0.1 rad/s for adaptive gripping
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
    print("         ANGLE-BASED GRIPPER CONTROL SYSTEM")
    print("=" * 60)
    
    # Stepper motor status with angle information
    cw_steps_per_deg = get_steps_per_degree(state.object_diameter_mm, 'cw')
    ccw_steps_per_deg = get_steps_per_degree(state.object_diameter_mm, 'ccw')
    
    print("ROTATION CONTROL (Angle-Based):")
    print(f"  Motor Position: M1={state.m1_target_pos:<4} | M2={state.m2_target_pos:<4}")
    print(f"  Current Angle:  {state.current_angle_deg:>6.1f}°")
    print(f"  Target Angle:   {state.target_angle_deg:>6.1f}°")
    print(f"  Object Diameter: {state.object_diameter_mm}mm")
    print(f"  Steps/Degree:   CW={cw_steps_per_deg:.3f} | CCW={ccw_steps_per_deg:.3f}")
    print(f"  Speed (steps/s): {state.speed:<4}")
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
    if state.sensor_connected:
        print(f"  Sensor FPS: {state.sensor_fps:.1f}")
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
    
    # Encoder status
    encoder_status = "Connected" if state.encoder_connected else "Disconnected"
    recording_status = "🔴 Recording" if state.is_recording else "⚪ Idle"
    data_count = len(state.recorded_data) if state.is_recording else 0
    
    print("ROTARY ENCODER:")
    print(f"  Status: {encoder_status}")
    print(f"  Recording: {recording_status} | Data Points: {data_count}")
    print(f"  Direction: CW (Fixed)")
    print(f"  Last Message: {state.last_encoder_msg}")
    
    print("-" * 60)
    
    # Controls
    print("CONTROLS:")
    print("  Rotation:  [A] CCW | [D] CW | [W/S] Speed | [Q/E] Angle ±5°")
    print("  Diameter:  [2/3/4/5] Set diameter (2-5mm)")
    print("  Gripper:   [O] Open | [C] Close (Adaptive)")
    print("  Sensor:    [B] Calibrate Baseline")
    print("  Encoder:   [Z] Zero")
    print("  General:   [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit")
    print("=" * 60)

# --- Keyboard Input Handling ---
def on_press(key):
    """Handles key press events for both stepper and gripper control."""
    try:
        # --- Angle-Based Stepper Motor Controls ---
        if key.char == 'a':  # Rotate CCW (counter-clockwise) by target angle
            send_angle_move_command(stepper_ser, state.target_angle_deg, 'ccw')
            # Start encoder recording for 5 seconds (CCW handled internally)
            if encoder_instrument and state.encoder_connected:
                start_encoder_recording('ccw')
        elif key.char == 'd':  # Rotate CW (clockwise) by target angle
            send_angle_move_command(stepper_ser, state.target_angle_deg, 'cw')
            # Start encoder recording for 5 seconds
            if encoder_instrument and state.encoder_connected:
                start_encoder_recording('cw')
        
        # --- Speed Control ---
        elif key.char == 'w':
            state.speed += 50
        elif key.char == 's':
            state.speed = max(50, state.speed - 50)
            
        # --- Target Angle Adjustment (Q/E keys) ---
        elif key.char == 'e':
            state.target_angle_deg += CONFIG["angle_increment_deg"]
            state.target_angle_deg = min(state.target_angle_deg, CONFIG["max_angle_deg"])
        elif key.char == 'q':
            state.target_angle_deg = max(CONFIG["angle_increment_deg"], 
                                       state.target_angle_deg - CONFIG["angle_increment_deg"])
        
        # --- Object Diameter Adjustment (Number keys 2-5) ---
        elif key.char == '2':
            state.object_diameter_mm = 2
        elif key.char == '3':
            state.object_diameter_mm = 3
        elif key.char == '4':
            state.object_diameter_mm = 4
        elif key.char == '5':
            state.object_diameter_mm = 5

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

        # --- Encoder Controls ---
        elif key.char == 'z':  # Zero encoder
            if encoder_instrument and state.encoder_connected:
                zero_encoder(encoder_instrument)

        # --- Utility Commands ---
        elif key.char == 'x':  # Reset stepper position to center and angle
            send_stepper_move_command(stepper_ser, CONFIG["initial_pos"], CONFIG["initial_pos"], CONFIG["homing_speed"])
            state.current_angle_deg = 0.0  # Reset angle tracking

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
    global stepper_ser, gripper_motor, gripper_motor_control, gripper_serial_port, sensor, encoder_instrument, listener
    
    stepper_ser = None
    gripper_motor = None
    gripper_motor_control = None
    gripper_serial_port = None
    sensor = None
    encoder_instrument = None
    listener = None
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
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

        # --- Setup Encoder Connection ---
        try:
            encoder_instrument = minimalmodbus.Instrument(CONFIG["encoder_port"], CONFIG["encoder_slave_address"])
            encoder_instrument.serial.baudrate = CONFIG["encoder_baudrate"]
            encoder_instrument.serial.bytesize = 8
            encoder_instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
            encoder_instrument.serial.stopbits = 1
            encoder_instrument.serial.timeout = 0.5
            encoder_instrument.mode = minimalmodbus.MODE_RTU
            
            # Test connection by reading encoder data
            test_angle, test_turns, test_single, test_raw, test_dir = read_encoder_data(encoder_instrument)
            if test_angle is not None:
                state.encoder_connected = True
                state.last_encoder_msg = f"Connected to {CONFIG['encoder_port']}"
                print(f"✓ Rotary encoder connected to {CONFIG['encoder_port']}")
                
                # Set encoder direction to CW (fixed)
                set_encoder_direction(encoder_instrument, False)  # False = CW
            else:
                print(f"❌ Could not read from encoder on {CONFIG['encoder_port']}")
                state.last_encoder_msg = "Connection failed - no data"
        except Exception as e:
            print(f"❌ Could not connect to encoder on {CONFIG['encoder_port']}: {e}")
            state.last_encoder_msg = f"Connection failed: {e}"


        # Start the background thread for reading stepper serial messages
        if stepper_ser and stepper_ser.is_open:
            reader = threading.Thread(target=stepper_reader, args=(stepper_ser, state))
            reader.daemon = True
            reader.start()

            # Perform initial homing of stepper motors
            print("Performing stepper motor homing...")
            send_stepper_command(stepper_ser, "<HOME>\n")
            time.sleep(10)  # Give homing time to complete

            # Move stepper to starting center position using homing speed
            send_stepper_move_command(stepper_ser, CONFIG["initial_pos"], CONFIG["initial_pos"], CONFIG["homing_speed"])

        # Start the background thread for reading sensor data
        if sensor and state.sensor_connected:
            sensor_reader_thread = threading.Thread(target=sensor_reader, args=(sensor, state))
            sensor_reader_thread.daemon = True
            sensor_reader_thread.start()
            print("✓ Sensor reader thread started")
            
            # Give sensor reader time to start and get initial readings
            print("Waiting for sensor to stabilize...")
            time.sleep(1.0)  # Allow sensor reader to start and get initial data

        # Start the background thread for reading encoder data
        if encoder_instrument and state.encoder_connected:
            encoder_reader_thread = threading.Thread(target=encoder_reader, args=(encoder_instrument, state))
            encoder_reader_thread.daemon = True
            encoder_reader_thread.start()
            print("✓ Encoder reader thread started")

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
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        
        last_update = 0
        while state.running:
            try:
                current_time = time.time()
                # Only update display every 0.5 seconds to reduce flickering
                if current_time - last_update >= 0.5:
                    update_display()
                    last_update = current_time
                time.sleep(0.1)  # Faster polling for keyboard input
            except KeyboardInterrupt:
                print("\n\nKeyboard interrupt received. Shutting down...")
                break
        
        # Stop the keyboard listener
        if listener:
            listener.stop()
            listener.join(timeout=2.0)
            print("✓ Keyboard listener stopped")
    
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
                # print("✓ Gripper opened")
            except:
                print("❌ Failed to open gripper")
        
        # Clean shutdown of stepper motor
        if stepper_ser and stepper_ser.is_open:
            try:
                # print("Stopping stepper motors...")
                send_stepper_command(stepper_ser, "<STOP>\n")
                time.sleep(0.1)
                send_stepper_move_command(stepper_ser, 0, 0, CONFIG["homing_speed"])
                time.sleep(2)
                stepper_ser.close()
                print("✓ Stepper motor serial port closed")
            except:
                pass
        
        # Clean shutdown of gripper motor
        if gripper_motor_control and gripper_motor:
            try:
                # print("Disabling gripper motor...")
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
                # print("Disconnecting sensor...")
                # Close image windows first
                close_sensor_windows()
                sensor.disconnect()
                print("✓ Sensor disconnected")
            except:
                pass
        
        # Clean shutdown of encoder
        if encoder_instrument and state.encoder_connected:
            try:
                # print("Disconnecting encoder...")
                # Stop any ongoing recording
                if state.is_recording:
                    stop_encoder_recording()
                print("✓ Encoder disconnected")
            except:
                pass
        
        # Stop the keyboard listener first
        if listener:
            try:
                # print("Stopping keyboard listener...")
                listener.stop()
                listener.join(timeout=2.0)
                print("✓ Keyboard listener stopped")
            except:
                pass
        
        # Force stop all background threads
        print("Stopping background threads...")
        state.running = False
        
        # Wait for threads to finish (with timeout)
        active_threads = threading.enumerate()
        main_thread = threading.main_thread()
        
        for thread in active_threads:
            if thread != main_thread and thread.is_alive():
                try:
                    print(f"Waiting for thread: {thread.name}")
                    thread.join(timeout=2.0)  # Wait up to 2 seconds per thread
                    if thread.is_alive():
                        print(f"⚠ Thread {thread.name} did not stop gracefully")
                except Exception as e:
                    print(f"⚠ Error stopping thread {thread.name}: {e}")
        
        # Close any remaining matplotlib windows
        try:
            plt.close('all')
            print("✓ Matplotlib windows closed")
        except:
            pass
        
        # Close any remaining OpenCV windows
        try:
            cv2.destroyAllWindows()
            print("✓ OpenCV windows closed")
        except:
            pass
        
        print("✓ Cleanup completed")
        print("Exiting program...")
        sys.exit(0)

if __name__ == "__main__":
    main()
