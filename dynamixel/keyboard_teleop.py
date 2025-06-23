import serial
import time
import sys
import os
import numpy as np
import msvcrt  # Import msvcrt for keyboard input

# Add paths before imports
sys.path.append(os.path.abspath('../damiao/DM_Control'))
sys.path.insert(0, os.path.abspath('../daimon'))  # Prepend DM-Tac folder to Python path

from DM_CAN import MotorControl, Motor as GripperMotor, DM_Motor_Type, Control_Type
from dmrobotics import Sensor

# Constants
GRIPPER_CLOSED_PERCENTAGE = 100  # Fully closed gripper
GRIPPER_OPEN_PERCENTAGE = 0  # Fully open gripper

# Constants for sensor feedback
INTENSITY_THRESHOLD = 0.1

# Initialize gripper motor
serial_port = serial.Serial('COM3', 921600, timeout=0.5)
gripper_motor = GripperMotor(DM_Motor_Type.DM4310, 0x01, 0x11)
gripper_control = MotorControl(serial_port)
gripper_control.addMotor(gripper_motor)
gripper_control.enable(gripper_motor)
gripper_control.switchControlMode(gripper_motor, Control_Type.POS_VEL)

# Initialize sensor
sensor = Sensor(0)  # Replace with appropriate serial ID
baseline_depth = sensor.getDepth()
baseline_max_intensity = np.max(baseline_depth)
baseline_pixel_sum = np.sum(baseline_depth)

"""Control the gripper based on the state and sensor feedback."""
def percentage_to_position(percentage, GRIPPER_MAX_POS=0.0, GRIPPER_MIN_POS=-1.38):
    """
    Convert closure percentage to radian position.
    """
    percentage = max(0, min(100, percentage))
    return GRIPPER_MAX_POS + (percentage / 100.0) * (GRIPPER_MIN_POS - GRIPPER_MAX_POS)

def safe_gripper_position(position, GRIPPER_MAX_POS=0.0, GRIPPER_MIN_POS=-1.38):
    """
    Ensure the gripper position stays within safe limits.
    """
    return max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))

def control_gripper(state):
    max_intensity = 0  # Initialize max_intensity to a default value

    if state == "grasp":
        # Check sensor feedback first
        depth = sensor.getDepth()
        adjusted_depth = depth - baseline_depth
        max_intensity = np.max(adjusted_depth)

        # If the threshold IS exceeded, command a STOP by holding the current position.
        if max_intensity > INTENSITY_THRESHOLD:
            current_pos = gripper_motor.getPosition()
            # Send a command to hold the current position with zero velocity.
            gripper_control.control_Pos_Vel(gripper_motor, current_pos, 2.0) 
        # Otherwise, continue closing.
        else:
            target_position = percentage_to_position(100) # Target fully closed 
            gripper_control.control_Pos_Vel(gripper_motor, target_position, 2.0)

    elif state == "release":
        # Fully open the gripper
        target_position = safe_gripper_position(percentage_to_position(0))
        gripper_control.control_Pos_Vel(gripper_motor, target_position, 2.0)
    
    return max_intensity

try:
    print("Keyboard Trigger Control")
    print("Press 'g' for GRASP, 'r' for RELEASE, 'q' to quit")
    print("Monitoring keyboard input...")
    
    current_state = "release"  # Start in release state
    
    while True:
        # Check for keyboard input
        if msvcrt.kbhit():
            key = msvcrt.getch().decode('utf-8').lower()
            
            if key == 'q':
                break
            elif key == 'g':
                current_state = "grasp"
                print("\nSwitched to GRASP mode")
            elif key == 'r':
                current_state = "release"
                print("\nSwitched to RELEASE mode")

        # Control gripper based on current state
        sensor_feedback = control_gripper(current_state)
        
        # Get gripper position in degrees
        gripper_pos = gripper_motor.getPosition()
        print(f"\rState: {current_state.upper()} | Gripper Position: {gripper_pos:.2f} | Max Intensity: {sensor_feedback:.2f}", end="")

        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    # Clean up
    gripper_control.disable(gripper_motor)
    print("Gripper Motor Disabled")
    serial_port.close()
    sensor.disconnect()
    print("Cleanup completed") 