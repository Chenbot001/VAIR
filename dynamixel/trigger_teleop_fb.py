from libgx.dynamixel_sdk import *  # Import the SDK
from libgx.motor import Motor as DynamixelMotor
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
DXL_ID = 7  # Dynamixel motor ID
COM_PORT = 'COM4'  # Communication port
BAUD_RATE = 1000000  # Baudrate for Dynamixel
TRIGGER_THRESHOLD = 160  # Threshold angle in degrees
GRIPPER_CLOSED_PERCENTAGE = 100  # Fully closed gripper
GRIPPER_OPEN_PERCENTAGE = 0  # Fully open gripper

# Constants for sensor feedback
INTENSITY_THRESHOLD = 0.1

# Initialize Dynamixel motor
portHandler = PortHandler(COM_PORT)
packetHandler = PacketHandler(2.0)

if not portHandler.openPort():
    print("Failed to open port")
    quit()

if not portHandler.setBaudRate(BAUD_RATE):
    print("Failed to set baudrate")
    quit()

trigger_motor = DynamixelMotor(DXL_ID, portHandler, packetHandler)
trigger_motor.torq_off()
trigger_motor.packet_handler.write1ByteTxRx(portHandler, DXL_ID, trigger_motor.addr_operating_mode, trigger_motor.pos_operating_mode)

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
    print("Monitoring trigger position. Press Ctrl+C to stop.")
    while True:
        # Read trigger motor position
        trigger_angle = trigger_motor.get_pos()

        # Determine gripper state based on trigger angle
        if trigger_angle > TRIGGER_THRESHOLD:
            state = "release"
        else:
            state = "grasp"

        sensor_feedback = control_gripper(state)
        # Get gripper position in degrees
        gripper_pos = gripper_motor.getPosition()
        print(f"\rTrigger Angle: {trigger_angle:.2f} | Gripper State: {state} | Gripper Position: {gripper_pos:.2f} | Max Intensity: {sensor_feedback:.2f}", end="")

        #time.sleep(0.01)
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    # Clean up
    trigger_motor.torq_off()
    gripper_control.disable(gripper_motor)
    print("Gripper Motor Disabled")
    portHandler.closePort()
    sensor.disconnect()