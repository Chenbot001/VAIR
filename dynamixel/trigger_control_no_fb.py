from libgx.dynamixel_sdk import *  # Import the SDK
from libgx.motor import Motor as DynamixelMotor
import serial
import time
import sys
import os

sys.path.append(os.path.abspath('../damiao/DM_Control'))

from DM_CAN import MotorControl, Motor as GripperMotor, DM_Motor_Type, Control_Type

# Constants
DXL_ID = 7  # Dynamixel motor ID
COM_PORT = 'COM4'  # Communication port
BAUD_RATE = 1000000  # Baudrate for Dynamixel
TRIGGER_THRESHOLD = 160  # Threshold angle in degrees
GRIPPER_CLOSED_PERCENTAGE = 100  # Fully closed gripper
GRIPPER_OPEN_PERCENTAGE = 0  # Fully open gripper

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

def control_gripper(state):
    """Control the gripper based on the state."""
    def percentage_to_position(percentage, GRIPPER_MAX_POS=0.0, GRIPPER_MIN_POS=-1.35):
        """
        Convert closure percentage to radian position.
        """
        percentage = max(0, min(100, percentage))
        return GRIPPER_MAX_POS + (percentage / 100.0) * (GRIPPER_MIN_POS - GRIPPER_MAX_POS)

    def safe_gripper_position(position, GRIPPER_MAX_POS=0.0, GRIPPER_MIN_POS=-1.35):
        """
        Ensure the gripper position stays within safe limits.
        """
        return max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))

    # Define gripper position limits
    GRIPPER_MAX_POS = 0.0  # Fully open position (0 rad)   
    GRIPPER_MIN_POS = -1.35  # Fully closed position (-1.35 rad)

    if state == "grasp":
        target_position = percentage_to_position(GRIPPER_CLOSED_PERCENTAGE)
    elif state == "release":
        target_position = percentage_to_position(GRIPPER_OPEN_PERCENTAGE)
    else:
        return

    # Enforce safety constraints
    target_position = safe_gripper_position(target_position)

    # Move the gripper to the target position
    gripper_control.control_Pos_Vel(gripper_motor, target_position, 2.0)

try:
    print("Monitoring trigger position. Press Ctrl+C to stop.")
    while True:
        # Read trigger motor position
        trigger_angle = trigger_motor.get_pos()

        # Determine gripper state based on trigger angle
        if trigger_angle < TRIGGER_THRESHOLD:
            state = "grasp"
        else:
            state = "release"

        control_gripper(state)

        print(f"\rTrigger Angle: {trigger_angle:.2f} degrees | Gripper State: {state}{' ' * 10}", end="")

        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    # Clean up
    trigger_motor.torq_off()
    gripper_control.disable(gripper_motor)
    portHandler.closePort()