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
    "gripper_default_open_percent": 50,  # Default "open" position (50% closed to save time)
    
    # === SENSOR CONFIGURATION ===
    "sensor_serial_id": 0,
    
    # === UR ROBOT CONFIGURATION ===
    "ur_robot_ip": "192.168.3.4",
    "ur_robot_step_size": 0.005,
    "ur_robot_time_duration": 0.2,
    "ur_robot_lookahead_time": 0.2,
    "ur_robot_gain": 300,
}

# Gripper position limits (in radians)
GRIPPER_MIN_POS = -1.37  # Fully closed position
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

# Centerline detection configuration
CENTERLINE_CONFIG = {
    "smoothing_alpha": 0.7,  # Exponential smoothing factor (0.0 = no smoothing, 1.0 = no new data)
    "history_length": 5,     # Number of frames to average for rolling smoothing
    "min_contour_area": 5,   # Minimum contour area for centerline detection
}

# Data recording configuration
RECORDING_CONFIG = {
    "snapshots_dir": os.path.join(os.path.dirname(__file__), "..", "data"),
    "default_session_name": "acupuncture",  # Base name for auto-generated session names
    "snapshot_interval": 0.1,  # Time between snapshots in seconds (10 Hz)
    "use_timestamp_session_name": True,  # If True, append timestamp to session name
    "create_session_subdirectories": True,  # If True, create subdirectory for each session
}

# Safety configuration
SAFETY_CONFIG = {
    "shear_force_threshold_n": 2.0,  # Deprecated - kept for backward compatibility
    "shear_x_threshold_n": 10.0,     # Maximum allowed shear force in X direction (Newtons)
    "shear_y_threshold_n": 1.0,     # Maximum allowed shear force in Y direction (Newtons)
    "emergency_open_percent": 50,    # Gripper position to open to during emergency
    "safety_check_enabled": True,    # Enable/disable safety monitoring
    "safety_cooldown_seconds": 2.0,  # Minimum time between safety triggers
}
