import math
from DM_CAN import *
import serial
import time
import numpy as np

# Gripper position limits (in radians)
GRIPPER_MIN_POS = -1.35  # Fully closed
GRIPPER_MAX_POS = 0.0    # Fully open

def safe_gripper_position(position):
    """
    Ensures the gripper position stays within safe limits.
    
    Args:
        position (float): Desired position in radians
        
    Returns:
        float: Position limited to safe range [-1.35, 0]
    """
    return max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))

Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)

serial_device = serial.Serial('COM3', 921600, timeout=0.5)
MotorControl1 = MotorControl(serial_device)
MotorControl1.addMotor(Motor1)
MotorControl1.enable(Motor1)
print("Motor Enabled")

MotorControl1.switchControlMode(Motor1, Control_Type.POS_VEL)

# Define a function to safely move gripper to a position
def move_gripper_to(position, velocity=3.0):
    """
    Move the gripper to a position safely within limits
    
    Args:
        position (float): Target position in radians
        velocity (float): Movement velocity
    """
    safe_pos = safe_gripper_position(position)
    if safe_pos != position:
        print(f"Warning: Requested position {position:.2f} limited to {safe_pos:.2f}")
        
    MotorControl1.control_Pos_Vel(Motor1, safe_pos, velocity)
    
# Test the gripper movement with safety limits
print(f"Moving gripper to open position (0 rad)...")
move_gripper_to(0, 2)
time.sleep(2)

print(f"Moving gripper to closed position (-1.35 rad)...")
move_gripper_to(-1.35, 2)
time.sleep(2)

print(f"Moving gripper to middle position (-0.675 rad)...")
move_gripper_to(-0.675, 2)
time.sleep(2)

move_gripper_to(0, 2)  # This should be limited to 0
time.sleep(2)

# 语句结束关闭串口
MotorControl1.disable(Motor1)
print("Motor Disabled")
serial_device.close()
print("Serial port closed.")
