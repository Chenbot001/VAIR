import pybullet as p
import pybullet_data
import numpy as np
from .DM_CAN import *
import math
import serial
import time
import keyboard
import ikpy.chain
from ikpy.chain import Chain
import ikpy


# Global variables
class MotorController:
    def __init__(self):
        self.Motor1 = None
        self.Motor2 = None
        self.Motor3 = None
        self.Motor4 = None
        self.Motor5 = None
        self.Motor6 = None
        self.Motor7 = None
        self.Motor_control = None

        self.motor_cmd_velocities = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
        self.arm_type = None

    def set_speed(self, speed: float):
        self.motor_cmd_velocities = np.ones([1, 7]) * speed

    def Motor_Init(self, port, rate, arm_type):
        self.arm_type = arm_type
        if arm_type == 'left':
            motors_info = [
                (DM_Motor_Type.DM4340, 0x01, 0x15),
                (DM_Motor_Type.DM4340, 0x02, 0x16),
                (DM_Motor_Type.DM4340, 0x03, 0x17),
                (DM_Motor_Type.DM4310, 0x04, 0x18),
                (DM_Motor_Type.DM4310, 0x05, 0x19),
                (DM_Motor_Type.DM4310, 0x06, 0x1A),
                (DM_Motor_Type.DM4310, 0x07, 0x1B),
            ]
        elif arm_type == 'right':
            motors_info = [
                (DM_Motor_Type.DM4340, 0x08, 0x1C),
                (DM_Motor_Type.DM4340, 0x09, 0x1D),
                (DM_Motor_Type.DM4340, 0x0A, 0x1E),
                (DM_Motor_Type.DM4310, 0x0B, 0x1F),
                (DM_Motor_Type.DM4310, 0x0C, 0x20),
                (DM_Motor_Type.DM4310, 0x0D, 0x21),
                (DM_Motor_Type.DM4310, 0x0E, 0x22),
            ]
        else:
            raise ValueError("Invalid arm type. Please choose 'left' or 'right'.")

        serial_device = serial.Serial(port, rate, timeout=0.5)
        self.Motor_control = MotorControl(serial_device)

        motors = []
        for motor_type, slave_id, master_id in motors_info:
            motor = Motor(motor_type, slave_id, master_id)
            self.Motor_control.addMotor(motor)
            motors.append(motor)

        self.Motor1, self.Motor2, self.Motor3, self.Motor4, self.Motor5, self.Motor6, self.Motor7 = motors

        for motor in motors:
            self.Motor_control.enable(motor)
            time.sleep(0.2)

        for motor in motors:
            self.Motor_control.switchControlMode(motor, Control_Type.POS_VEL)
            self.Motor_control.save_motor_param(motor)
            time.sleep(0.2)

        print("Init Successfully")

    def get_all_positions(self):
        global Motor1, Motor2, Motor3, Motor4, Motor5, Motor6, Motor7
        positions = [
            self.Motor1.getPosition(),
            self.Motor2.getPosition(),
            self.Motor3.getPosition(),
            self.Motor4.getPosition(),
            self.Motor5.getPosition(),
            self.Motor6.getPosition(),
            self.Motor7.getPosition()
        ]
        return np.array(positions)

    def Motor_all_control(self, positions, velocities):
        """
        Send position and velocity commands to all seven motors.

        :param positions: A list or array of seven positions.
        :param velocities: A list or array of seven velocities.
        """

        motors = [self.Motor1, self.Motor2, self.Motor3, self.Motor4, self.Motor5, self.Motor6, self.Motor7]

        for motor, position, velocity in zip(motors, positions, velocities):
            self.Motor_control.control_Pos_Vel(motor, position, velocity)
            time.sleep(0.002)  # 2 ms delay

    def Disable_all_motors(self):
        """
        Disable all seven motors.
        """

        motors = [self.Motor1, self.Motor2, self.Motor3, self.Motor4, self.Motor5, self.Motor6, self.Motor7]

        for motor in motors:
            self.Motor_control.disable(motor)
            time.sleep(0.1)  # Ensure each motor is disabled properly

    def Cmd_Limit_Clip(self, motor_now_positions):
        if self.arm_type == 'left':
            joint_limits = np.array([
                [-1.57, 0.2],  # 第1个关节范围
                [-0.1, 1.2],  # 第2个关节范围
                [-1.0, 1.0],  # 第3个关节范围
                [-2.6, 0],  # 第4个关节范围
                [-1.57, 1.57],  # 第5个关节范围
                [-1.0, 0.8],  # 第6个关节范围
                [-3.14, 3.14],  # 第7个关节范围
            ])
        elif self.arm_type == 'right':
            joint_limits = np.array([
                [-1.57, 0.2],  # 第1个关节范围
                [-1.2, 0.1],  # 第2个关节范围
                [-1.0, 1.0],  # 第3个关节范围
                [0, 2.6],  # 第4个关节范围
                [-1.57, 1.57],  # 第5个关节范围
                [-1.0, 0.8],  # 第6个关节范围
                [-3.14, 3.14],  # 第7个关节范围
            ])
        else:
            raise ValueError("Invalid arm type. Please choose 'left' or 'right'.")

        return np.clip(motor_now_positions, a_min=joint_limits[:, 0], a_max=joint_limits[:, 1])

    def Cmd_Safe_Clip(self, motor_cmd_positions):
        if self.arm_type == 'left':
            joint_limits = [
                (-1.57, 0.2),  # 第1个关节范围
                (-0.1, 1.2),  # 第2个关节范围
                (-1.0, 1.0),  # 第3个关节范围
                (-2.6, 0),  # 第4个关节范围
                (-1.57, 1.57),  # 第5个关节范围
                (-1.0, 0.8),  # 第6个关节范围
                (-3.14, 3.14),  # 第7个关节范围
            ]
        elif self.arm_type == 'right':
            joint_limits = [
                (-1.57, 0.2),  # 第1个关节范围
                (-1.2, 0.1),  # 第2个关节范围
                (-1.0, 1.0),  # 第3个关节范围
                (0, 2.6),  # 第4个关节范围
                (-1.57, 1.57),  # 第5个关节范围
                (-1.0, 0.8),  # 第6个关节范围
                (-3.14, 3.14),  # 第7个关节范围
            ]
        else:
            raise ValueError("Invalid arm type. Please choose 'left' or 'right'.")

        # 检查每个关节的位置是否在各自的范围内
        for i, (cmd, (lower, upper)) in enumerate(zip(motor_cmd_positions, joint_limits)):
            if not (lower <= cmd <= upper):
                print(f"Joint {i + 1} out of range: {cmd}")
                return None

        # 如果所有值都在范围内，返回原数组
        return motor_cmd_positions

    def motor_safe_control(self, motor_cmd_positions):
        motor_cmd_positions = motor_cmd_positions[1:]
        motor_safe_cmd = self.Cmd_Safe_Clip(motor_cmd_positions)
        self.Motor_all_control(motor_safe_cmd, self.motor_cmd_velocities)
