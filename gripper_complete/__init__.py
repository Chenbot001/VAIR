"""
Gripper Control System - Modular Architecture Package

This package contains a refactored, modular version of the original 
gripper_angle_control_encoder.py script.

Modules:
- main: Main application entry point
- config: Centralized configuration
- hardware_manager: Motor communications
- sensor_manager: Sensor interactions
- utils: Helper functions
"""

__version__ = "1.0.0"
__author__ = "Gripper Control System Team"

from .main import GripperControlSystem, SystemState
from .hardware_manager import HardwareManager, StepperMotorManager, GripperMotorManager
from .sensor_manager import SensorManager, DaimonManager, RotaryEncoderManager
from .utils import (
    get_steps_per_degree, angle_to_steps, steps_to_angle,
    calculate_motor_positions, unwrap_angles,
    percentage_to_position, position_to_percentage,
    save_encoder_data_to_csv, create_encoder_plot
)

__all__ = [
    'GripperControlSystem',
    'SystemState', 
    'HardwareManager',
    'StepperMotorManager',
    'GripperMotorManager',
    'SensorManager',
    'DaimonManager',
    'RotaryEncoderManager',
    'get_steps_per_degree',
    'angle_to_steps',
    'steps_to_angle',
    'calculate_motor_positions',
    'unwrap_angles',
    'percentage_to_position',
    'position_to_percentage',
    'save_encoder_data_to_csv',
    'create_encoder_plot'
]
