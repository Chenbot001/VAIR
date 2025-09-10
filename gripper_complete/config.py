"""
Configuration file for the Gripper Control System
Contains all parameters, constants, and calibration data
"""

import os

# --- Main Configuration ---
CONFIG = {
    # === PORT ASSIGNMENTS ===
    # Update these ports as needed for your hardware setup
    "stepper_port": "COM10",        # Stepper motor controller port
    "gripper_port": "COM3",        # DM gripper motor port
    
    # === STEPPER MOTOR CONFIGURATION ===
    "stepper_baud_rate": 115200,
    "microsteps": 16,
    "max_steps": 1000,
    "initial_pos": 500,
    "initial_speed": 50,
    "homing_speed": 500,  # Fast speed for homing and initialization operations
    
    # === CONTROL MODE CONFIGURATION ===
    "initial_diameter_mm": 2,
    
    # === FIXED CONTROL PARAMETERS ===
    "fixed_motor_speed": 50,       # Fixed motor speed (steps/sec)
    "fixed_target_steps": 20,      # Fixed target steps for step-based control
    
    # === GRIPPER MOTOR CONFIGURATION ===
    "gripper_baud_rate": 921600,
    "gripper_motor_id": 0x01,
    "gripper_can_id": 0x11,
    
    # === SENSOR CONFIGURATION ===
    "sensor_serial_id": 0,
    
    # === UR ROBOT CONFIGURATION ===
    "ur_robot_ip": "192.168.3.4",
    "ur_robot_init_pose": [0.43, -0.11, 0.40, 
                          -1.1113946900002296, 2.9307050851101204, -0.07006688172175954],
    "ur_robot_step_size": 0.005,
    "ur_robot_time_duration": 0.1,
    "ur_robot_lookahead_time": 0.2,
    "ur_robot_gain": 100,
}

# Gripper position limits (in radians)
GRIPPER_MIN_POS = -1.38  # Fully closed position
GRIPPER_MAX_POS = 0.0    # Fully open position

# Adaptive gripping configuration
ADAPTIVE_GRIPPING_CONFIG = {
    "enabled": True,
    "threshold": 0.5,  # Lower threshold for more sensitive detection
    "opening_velocity": 0.3,  # rad/s for opening
    "closing_velocity": 0.1,  # rad/s for adaptive gripping
}

# Display configuration
DISPLAY_CONFIG = {
    "update_interval_seconds": 0.5,
    "sensor_fps_calculation_interval": 1.0,
    "show_sensor_images": True,  # Set to False to disable sensor image windows
    "sensor_image_update_rate_hz": 10,  # How often to update sensor images (lower = less CPU usage)
}

# Manual tilt configuration for calibration purposes
MANUAL_TILT_CONFIG = {
    "enabled": True,  # When True, uses manual tilt instead of calculated centerline angle
    "current_value": 0,  # Current manual tilt value in degrees
    "valid_values": [0, 10, 20, 30],  # Valid tilt values that can be toggled
}


