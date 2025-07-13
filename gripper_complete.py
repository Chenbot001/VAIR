#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Comprehensive Gripper Control System
====================================
This program integrates three components:
1. Dynamixel trigger control for gripper opening/closing
2. Stepper motor control for catheter rotation
3. Daimon visuotactile sensor feedback for adaptive grasping

Hardware Setup:
- Gripper opening/closing: Damiao servo motor
- Catheter rotation: Linear travel stepper motors (rack and pinion)
- Force feedback: Daimon visuotactile sensors
- Teleoperation: Dynamixel motors from Gello device (isolated handle/trigger)

Controls:
- Trigger position: Controls gripper open/close
- Keyboard: Controls stepper motor rotation (A/D keys)
- Sensors: Provide adaptive feedback during grasping
"""

import sys
import os
import time
import threading
import serial
import numpy as np
import cv2
import signal
from pynput import keyboard
from enum import Enum

# Add paths for local imports
sys.path.append(os.path.abspath('dynamixel/libgx'))
sys.path.append(os.path.abspath('damiao/DM_Control'))
sys.path.append(os.path.abspath('daimon'))

# Import local modules
try:
    from dynamixel_sdk import *
    from motor import Motor as DynamixelMotor
    from DM_CAN import MotorControl, Motor as GripperMotor, DM_Motor_Type, Control_Type
    from dmrobotics import Sensor, put_arrows_on_image
except ImportError as e:
    print(f"Warning: Could not import local modules: {e}")
    print("Make sure all required modules are available in the subdirectories")
    # Define placeholder classes for type checking
    class DynamixelMotor:
        pass
    class GripperMotor:
        pass
    class MotorControl:
        pass
    class Sensor:
        pass

class GripperState(Enum):
    """Enumeration for gripper states"""
    OPEN = "open"
    CLOSED = "closed"
    ADAPTIVE = "adaptive"

class StepperState(Enum):
    """Enumeration for stepper motor states"""
    IDLE = "idle"
    EXTENDING = "extending"
    RETRACTING = "retracting"

class GripperController:
    """Main controller class for the comprehensive gripper system"""
    
    def __init__(self):
        # Configuration
        self.config = {
            # Dynamixel trigger settings
            "trigger_id": 7,
            "trigger_port": "COM4",
            "trigger_baud": 1000000,
            "trigger_threshold": 160,  # degrees
            
            # Gripper motor settings
            "gripper_port": "COM3",
            "gripper_baud": 921600,
            "gripper_open_percent": 0,
            "gripper_closed_percent": 100,
            "gripper_max_pos": 0.0,
            "gripper_min_pos": -1.38,
            
            # Stepper motor settings
            "stepper_port": "COM8",
            "stepper_baud": 115200,
            "stepper_microsteps": 16,
            "stepper_max_steps": 1000,
            "stepper_initial_pos": 500,
            "stepper_speed": 100,
            "stepper_step_size": 50,
            
            # Sensor settings
            "sensor_intensity_threshold": 0.1,
            "sensor_display_scale": 20,
            "sensor_depth_scale": 0.25,
        }
        
        # State tracking
        self.gripper_state = GripperState.OPEN
        self.stepper_state = StepperState.IDLE
        self.running = True
        self.sensor_baseline = None
        
        # Initialize components
        self.trigger_motor = None
        self.gripper_motor = None
        self.gripper_control = None
        self.stepper_serial = None
        self.sensor = None
        
        # Threading
        self.sensor_thread: threading.Thread = None  # type: ignore
        self.display_thread: threading.Thread = None  # type: ignore
        
    def initialize_trigger(self):
        """Initialize the Dynamixel trigger motor"""
        try:
            port_handler = PortHandler(self.config["trigger_port"])
            packet_handler = PacketHandler(2.0)
            
            if not port_handler.openPort():
                raise Exception(f"Failed to open trigger port {self.config['trigger_port']}")
            
            if not port_handler.setBaudRate(self.config["trigger_baud"]):
                raise Exception(f"Failed to set trigger baudrate {self.config['trigger_baud']}")
            
            self.trigger_motor = DynamixelMotor(
                self.config["trigger_id"], 
                port_handler, 
                packet_handler
            )
            
            # Configure trigger motor
            self.trigger_motor.torq_off()
            self.trigger_motor.packet_handler.write1ByteTxRx(
                port_handler, 
                self.config["trigger_id"], 
                self.trigger_motor.addr_operating_mode, 
                self.trigger_motor.pos_operating_mode
            )
            
            print(f"✓ Trigger motor initialized on {self.config['trigger_port']}")
            return True
            
        except Exception as e:
            print(f"✗ Failed to initialize trigger motor: {e}")
            return False
    
    def initialize_gripper(self):
        """Initialize the Damiao gripper motor"""
        try:
            serial_port = serial.Serial(
                self.config["gripper_port"], 
                self.config["gripper_baud"], 
                timeout=0.5
            )
            
            self.gripper_motor = GripperMotor(DM_Motor_Type.DM4310, 0x01, 0x11)
            self.gripper_control = MotorControl(serial_port)
            self.gripper_control.addMotor(self.gripper_motor)
            self.gripper_control.enable(self.gripper_motor)
            self.gripper_control.switchControlMode(self.gripper_motor, Control_Type.POS_VEL)
            
            print(f"✓ Gripper motor initialized on {self.config['gripper_port']}")
            return True
            
        except Exception as e:
            print(f"✗ Failed to initialize gripper motor: {e}")
            return False
    
    def initialize_stepper(self):
        """Initialize the stepper motor serial connection"""
        try:
            self.stepper_serial = serial.Serial(
                self.config["stepper_port"], 
                self.config["stepper_baud"], 
                timeout=1
            )
            
            # Wait for Arduino to boot
            time.sleep(2)
            
            # Perform initial homing
            print("Performing stepper motor homing...")
            self.send_stepper_command("<HOME>\n")
            time.sleep(10)  # Give homing time to complete
            
            # Move to starting position
            self.send_stepper_move(
                self.config["stepper_initial_pos"], 
                self.config["stepper_initial_pos"], 
                self.config["stepper_speed"]
            )
            
            print(f"✓ Stepper motors initialized on {self.config['stepper_port']}")
            return True
            
        except Exception as e:
            print(f"✗ Failed to initialize stepper motors: {e}")
            return False
    
    def initialize_sensor(self):
        """Initialize the Daimon visuotactile sensor"""
        try:
            self.sensor = Sensor(0)  # Use first sensor
            time.sleep(2.5)  # Allow sensor to initialize
            
            # Get baseline readings
            baseline_depth = self.sensor.getDepth()
            self.sensor_baseline = {
                'depth': baseline_depth,
                'max_intensity': np.max(baseline_depth),
                'pixel_sum': np.sum(baseline_depth)
            }
            
            print("✓ Visuotactile sensor initialized")
            return True
            
        except Exception as e:
            print(f"✗ Failed to initialize sensor: {e}")
            return False
    
    def percentage_to_position(self, percentage):
        """Convert closure percentage to radian position"""
        percentage = max(0, min(100, percentage))
        return (self.config["gripper_max_pos"] + 
                (percentage / 100.0) * 
                (self.config["gripper_min_pos"] - self.config["gripper_max_pos"]))
    
    def safe_gripper_position(self, position):
        """Ensure gripper position stays within safe limits"""
        return max(self.config["gripper_min_pos"], 
                  min(self.config["gripper_max_pos"], position))
    
    def control_gripper(self, state):
        """Control the gripper based on state and sensor feedback"""
        max_intensity = 0
        
        if state == GripperState.CLOSED:
            # Fully close the gripper
            target_position = self.safe_gripper_position(
                self.percentage_to_position(self.config["gripper_closed_percent"])
            )
            self.gripper_control.control_Pos_Vel(self.gripper_motor, target_position, 2.0)
            
        elif state == GripperState.OPEN:
            # Fully open the gripper
            target_position = self.safe_gripper_position(
                self.percentage_to_position(self.config["gripper_open_percent"])
            )
            self.gripper_control.control_Pos_Vel(self.gripper_motor, target_position, 2.0)
            
        elif state == GripperState.ADAPTIVE:
            # Adaptive grasping with sensor feedback
            if self.sensor and self.sensor_baseline:
                depth = self.sensor.getDepth()
                adjusted_depth = depth - self.sensor_baseline['depth']
                max_intensity = np.max(adjusted_depth)
                
                # If threshold exceeded, hold current position
                if max_intensity > self.config["sensor_intensity_threshold"]:
                    current_pos = self.gripper_motor.getPosition()
                    self.gripper_control.control_Pos_Vel(self.gripper_motor, current_pos, 2.0)
                else:
                    # Continue closing
                    target_position = self.safe_gripper_position(
                        self.percentage_to_position(self.config["gripper_closed_percent"])
                    )
                    self.gripper_control.control_Pos_Vel(self.gripper_motor, target_position, 2.0)
        
        return max_intensity
    
    def send_stepper_command(self, command):
        """Send a command to the stepper motors"""
        if self.stepper_serial and self.stepper_serial.is_open:
            self.stepper_serial.write(command.encode('utf-8'))
    
    def send_stepper_move(self, m1_pos, m2_pos, speed):
        """Send a move command to the stepper motors"""
        # Clamp positions to physical limits
        m1_pos = max(0, min(m1_pos, self.config["stepper_max_steps"]))
        m2_pos = max(0, min(m2_pos, self.config["stepper_max_steps"]))
        
        command = f"<{int(m1_pos)},{int(m2_pos)},{int(speed)},{self.config['stepper_microsteps']}>\n"
        self.send_stepper_command(command)
    
    def rotate_catheter(self, direction):
        """Rotate the catheter using stepper motors"""
        if direction == "right":
            # Extend one motor, retract the other for rotation
            current_m1 = self.config["stepper_initial_pos"] + self.config["stepper_step_size"]
            current_m2 = self.config["stepper_initial_pos"] - self.config["stepper_step_size"]
            self.send_stepper_move(current_m1, current_m2, self.config["stepper_speed"])
        elif direction == "left":
            # Retract one motor, extend the other for rotation
            current_m1 = self.config["stepper_initial_pos"] - self.config["stepper_step_size"]
            current_m2 = self.config["stepper_initial_pos"] + self.config["stepper_step_size"]
            self.send_stepper_move(current_m1, current_m2, self.config["stepper_speed"])
    
    def on_key_press(self, key):
        """Handle keyboard input for stepper motor control"""
        try:
            if key.char == 'd':  # Rotate right
                self.rotate_catheter("right")
            elif key.char == 'a':  # Rotate left
                self.rotate_catheter("left")
            elif key.char == 's':  # Stop stepper motors
                self.send_stepper_command("<STOP>\n")
            elif key.char == 'r':  # Reset stepper position
                self.send_stepper_move(
                    self.config["stepper_initial_pos"], 
                    self.config["stepper_initial_pos"], 
                    self.config["stepper_speed"]
                )
                
        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.esc:
                self.running = False
                return False
    
    def sensor_monitor_thread(self):
        """Background thread for sensor monitoring"""
        while self.running:
            if self.sensor:
                try:
                    # Get sensor data for display
                    depth = self.sensor.getDepth()
                    deformation = self.sensor.getDeformation2D()
                    shear = self.sensor.getShear()
                    
                    # Process for adaptive grasping
                    if self.gripper_state == GripperState.ADAPTIVE:
                        self.control_gripper(GripperState.ADAPTIVE)
                        
                except Exception as e:
                    print(f"Sensor error: {e}")
            
            time.sleep(0.01)  # 100Hz update rate
    
    def display_thread(self):
        """Background thread for status display"""
        while self.running:
            try:
                # Clear screen
                os.system('cls' if os.name == 'nt' else 'clear')
                
                print("=== Comprehensive Gripper Control System ===")
                print(f"Gripper State: {self.gripper_state.value}")
                print(f"Stepper State: {self.stepper_state.value}")
                
                # Display trigger angle
                if self.trigger_motor:
                    trigger_angle = self.trigger_motor.get_pos()
                    print(f"Trigger Angle: {trigger_angle:.2f}°")
                
                # Display gripper position
                if self.gripper_motor:
                    gripper_pos = self.gripper_motor.getPosition()
                    print(f"Gripper Position: {gripper_pos:.3f} rad")
                
                # Display sensor feedback
                if self.sensor and self.sensor_baseline:
                    depth = self.sensor.getDepth()
                    adjusted_depth = depth - self.sensor_baseline['depth']
                    max_intensity = np.max(adjusted_depth)
                    print(f"Sensor Max Intensity: {max_intensity:.3f}")
                
                print("\nControls:")
                print("  Trigger: Controls gripper open/close")
                print("  [A/D]: Rotate catheter left/right")
                print("  [S]: Stop stepper motors")
                print("  [R]: Reset stepper position")
                print("  [ESC]: Quit")
                print("=" * 50)
                
            except Exception as e:
                print(f"Display error: {e}")
            
            time.sleep(0.1)  # 10Hz update rate
    
    def main_control_loop(self):
        """Main control loop for trigger-based gripper control"""
        print("Starting main control loop...")
        
        try:
            while self.running:
                # Read trigger motor position
                if self.trigger_motor:
                    trigger_angle = self.trigger_motor.get_pos()
                    
                    # Determine gripper state based on trigger angle
                    if trigger_angle > self.config["trigger_threshold"]:
                        self.gripper_state = GripperState.OPEN
                    else:
                        self.gripper_state = GripperState.ADAPTIVE
                    
                    # Control gripper based on state
                    sensor_feedback = self.control_gripper(self.gripper_state)
                
                time.sleep(0.01)  # 100Hz control rate
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        except Exception as e:
            print(f"Control loop error: {e}")
    
    def cleanup(self):
        """Clean up all resources"""
        print("\nCleaning up...")
        
        # Stop stepper motors
        if self.stepper_serial:
            self.send_stepper_command("<STOP>\n")
            time.sleep(0.1)
            self.send_stepper_move(0, 0, 500)
            time.sleep(2)
            self.stepper_serial.close()
        
        # Disable gripper motor
        if self.gripper_control and self.gripper_motor:
            self.gripper_control.disable(self.gripper_motor)
        
        # Disable trigger motor
        if self.trigger_motor:
            self.trigger_motor.torq_off()
        
        # Disconnect sensor
        if self.sensor:
            self.sensor.disconnect()
        
        print("Cleanup completed")
    
    def run(self):
        """Main run method"""
        print("Initializing Comprehensive Gripper Control System...")
        
        # Initialize all components
        if not all([
            self.initialize_trigger(),
            self.initialize_gripper(),
            self.initialize_stepper(),
            self.initialize_sensor()
        ]):
            print("Failed to initialize one or more components")
            return
        
        print("All components initialized successfully!")
        
        # Start background threads
        self.sensor_thread = threading.Thread(target=self.sensor_monitor_thread)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()
        
        self.display_thread = threading.Thread(target=self.display_thread)
        self.display_thread.daemon = True
        self.display_thread.start()
        
        # Start keyboard listener
        keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        keyboard_listener.start()
        
        # Run main control loop
        try:
            self.main_control_loop()
        finally:
            self.cleanup()

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully"""
    print('\nKeyboard interrupt received. Shutting down gracefully...')
    sys.exit(0)

if __name__ == "__main__":
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and run controller
    controller = GripperController()
    controller.run() 