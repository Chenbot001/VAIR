"""
Configuration file for the Gripper Control System
Contains all parameters, constants, and calibration data
"""

import os

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

# --- Main Configuration ---
CONFIG = {
    # Stepper motor configuration
    "stepper_port": "COM9",
    "stepper_baud_rate": 115200,
    "microsteps": 16,
    "max_steps": 1000,
    "initial_pos": 500,
    "initial_speed": 50,
    "homing_speed": 500,  # Fast speed for homing and initialization operations
    
    # Control mode configuration
    "min_diameter_mm": 0.2,
    "max_diameter_mm": 5,
    "initial_diameter_mm": 2,
    
    # Angle-based control configuration (for diameters 2-5mm)
    "angle_increment_deg": 5,
    "initial_target_angle_deg": 90,  # Default target angle setting
    "max_angle_deg": 180,  # Maximum rotation angle from center
    
    # Step-based control configuration (for diameters ≤1mm)
    "step_increment": 5,
    "initial_target_steps": 10,
    "max_steps_movement": 500,
    
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

# Adaptive gripping configuration
ADAPTIVE_GRIPPING_CONFIG = {
    "enabled": True,
    "threshold": 0.5,  # Lower threshold for more sensitive detection
    "opening_velocity": 0.3,  # rad/s for opening
    "closing_velocity": 0.1,  # rad/s for adaptive gripping
}

# Recording configuration
RECORDING_CONFIG = {
    "duration_seconds": 5.0,
    "sampling_rate_hz": 20,  # 20Hz reading rate
    "error_threshold_degrees": 10.0,  # ±10 degrees acceptable error
}

# Display configuration
DISPLAY_CONFIG = {
    "update_interval_seconds": 0.5,
    "sensor_fps_calculation_interval": 1.0,
    "show_sensor_images": True,
}

# File paths
PATHS = {
    "encoder_data_dir": "rotary_encoder",
    "encoder_plot_dir": "rotary_encoder/plot",
    "encoder_csv_file": "encoder_data.csv",
}
