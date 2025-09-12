"""
Hardware Manager for the Gripper Control System
Manages all motor communications (Stepper and Gripper)
"""

import serial
import time
import threading
import sys
import os

# Add paths for local imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'damiao', 'DM_Control'))

from config import CONFIG, GRIPPER_MIN_POS, GRIPPER_MAX_POS, ADAPTIVE_GRIPPING_CONFIG
from utils import (
    calculate_motor_positions, 
    percentage_to_position, 
    position_to_percentage, 
    safe_gripper_position,
    angle_to_steps
)

# Import DM motor control from the damiao folder
try:
    from DM_CAN import *
    DM_AVAILABLE = True
except ImportError:
    print("Warning: DM motor control not available. Gripper functions disabled.")
    DM_AVAILABLE = False


class StepperMotorManager:
    """Manages stepper motor communication and control"""
    
    def __init__(self):
        self.serial_connection = None
        self.connected = False
        self.last_message = ""
        self.m1_target_pos = CONFIG["initial_pos"]
        self.m2_target_pos = CONFIG["initial_pos"]
        self.speed = CONFIG["initial_speed"]
        self.current_angle_deg = 0.0
        
    def connect(self):
        """Initialize stepper motor connection"""
        try:
            self.serial_connection = serial.Serial(
                CONFIG["stepper_port"], 
                CONFIG["stepper_baud_rate"], 
                timeout=1
            )
            self.connected = True
            self.last_message = f"Connected to {CONFIG['stepper_port']}"
            print(f"✓ Stepper motor connected to {CONFIG['stepper_port']}")
            time.sleep(2)  # Give Arduino time to boot
            
            # Perform initial homing and centering (like the original script)
            if self.connected:
                print("Performing stepper motor homing...")
                self.send_command("<HOME>\n")
                time.sleep(10)  # Give homing time to complete
                
                # Move to starting center position using homing speed
                self.send_move_command(
                    CONFIG["initial_pos"], 
                    CONFIG["initial_pos"], 
                    CONFIG["homing_speed"]
                )
                time.sleep(2)  # Give time for centering to complete
                
                self.last_message = "Stepper motors homed and centered"
                print("✓ Stepper motors homed and centered")
            
            return True
        except serial.SerialException as e:
            print(f"❌ Could not connect to stepper motor on {CONFIG['stepper_port']}: {e}")
            self.last_message = f"Connection failed: {e}"
            return False
    
    def disconnect(self):
        """Close stepper motor connection"""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.send_command("<STOP>\n")
                time.sleep(0.1)
                self.send_move_command(0, 0, CONFIG["homing_speed"])
                time.sleep(2)
                self.serial_connection.close()
                print("✓ Stepper motor serial port closed")
            except:
                pass
        self.connected = False
    
    def send_command(self, command):
        """Send a command to the Arduino"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(command.encode('utf-8'))
    
    def send_move_command(self, m1, m2, speed):
        """Send a standard stepper motor move command"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return
        
        # Clamp position to physical limits as a safety measure
        m1 = max(0, min(m1, CONFIG["max_steps"]))
        m2 = max(0, min(m2, CONFIG["max_steps"]))
        command = f"<{int(m1)},{int(m2)},{int(speed)},{CONFIG['microsteps']}>\n"
        self.send_command(command)
        
        # Update the target positions
        self.m1_target_pos = m1
        self.m2_target_pos = m2
    
    def get_current_angle_stable(self):
        """
        Get a stable reading of the current angle without any race conditions.
        This method returns the angle before any pending updates.
        """
        return self.current_angle_deg
    
    def send_angle_move_command(self, angle_deg, direction, object_diameter_mm):
        """
        Send a move command based on angle and direction.
        
        Args:
            angle_deg: Target angle increment in degrees
            direction: 'cw' or 'ccw'
            object_diameter_mm: Object diameter for calibration
            
        Returns:
            float: Initial angle before the movement starts
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            return self.current_angle_deg
        
        # Capture initial angle before any updates
        initial_angle = self.current_angle_deg
        
        # Calculate new motor positions
        new_m1, new_m2 = calculate_motor_positions(
            angle_deg, direction, 
            self.m1_target_pos, self.m2_target_pos, 
            object_diameter_mm
        )
        
        # Send the move command
        self.send_move_command(new_m1, new_m2, self.speed)
        
        # Update current angle based on actual movement
        if direction == 'ccw':
            self.current_angle_deg -= angle_deg  # CCW decreases angle (negative direction)
        else:  # cw
            self.current_angle_deg += angle_deg  # CW increases angle (positive direction)
        
        # Keep angle within reasonable bounds for display
        if self.current_angle_deg > 360:
            self.current_angle_deg -= 360
        elif self.current_angle_deg < -360:
            self.current_angle_deg += 360
            
        return initial_angle
    
    def send_step_move_command(self, steps, direction):
        """
        Send a move command based on direct step count and direction.
        
        Args:
            steps: Number of steps to move
            direction: 'cw' or 'ccw'
            
        Returns:
            float: Initial angle before the movement starts
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            return self.current_angle_deg
        
        # Capture initial angle before any updates
        initial_angle = self.current_angle_deg
        
        # Calculate new motor positions based on direction
        if direction == 'ccw':  # A key - counter-clockwise
            new_m1 = self.m1_target_pos + steps
            new_m2 = self.m2_target_pos - steps
        else:  # direction == 'cw' - D key - clockwise
            new_m1 = self.m1_target_pos - steps
            new_m2 = self.m2_target_pos + steps
        
        # Clamp to physical limits
        new_m1 = max(0, min(CONFIG["max_steps"], new_m1))
        new_m2 = max(0, min(CONFIG["max_steps"], new_m2))
        
        # Send the move command
        self.send_move_command(new_m1, new_m2, self.speed)
        
        # Update current angle based on actual movement (approximate)
        # For step mode, we'll track a rough angle estimate
        if direction == 'ccw':
            self.current_angle_deg -= (steps / 10.0)  # Rough approximation
        else:  # cw
            self.current_angle_deg += (steps / 10.0)  # Rough approximation
        
        # Keep angle within reasonable bounds for display
        if self.current_angle_deg > 360:
            self.current_angle_deg -= 360
        elif self.current_angle_deg < -360:
            self.current_angle_deg += 360
            
        return initial_angle
    
    def home_motors(self):
        """Perform homing operation"""
        if self.connected:
            print("Performing stepper motor homing...")
            self.send_command("<HOME>\n")
            time.sleep(10)  # Give homing time to complete
            
            # Move to starting center position
            self.send_move_command(
                CONFIG["initial_pos"], 
                CONFIG["initial_pos"], 
                CONFIG["homing_speed"]
            )
    
    def stop_motors(self):
        """Stop all stepper motors"""
        self.send_command("<STOP>\n")
        self.last_message = "STOP command sent"
    
    def reset_to_center(self):
        """Reset stepper position to center and zero angle tracking"""
        self.send_move_command(
            CONFIG["initial_pos"], 
            CONFIG["initial_pos"], 
            CONFIG["homing_speed"]
        )
        self.current_angle_deg = 0.0  # Reset angle tracking
    
    def start_reader_thread(self, state):
        """Start background thread for reading stepper serial messages"""
        if self.serial_connection and self.serial_connection.is_open:
            reader = threading.Thread(target=self._reader_loop, args=(state,))
            reader.daemon = True
            reader.start()
            return reader
        return None
    
    def _reader_loop(self, state):
        """Background thread for reading stepper serial messages"""
        while state.running:
            try:
                if self.serial_connection.in_waiting > 0:
                    response = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        self.last_message = response
            except serial.SerialException:
                self.last_message = "ERR: Stepper serial disconnected."
                self.connected = False
                break
            time.sleep(0.05)


class GripperMotorManager:
    """Manages gripper motor communication and control"""
    
    def __init__(self):
        self.motor = None
        self.motor_control = None
        self.serial_port = None
        self.connected = False
        self.last_message = ""
        self.gripper_closure_percent = 0
        self.gripper_position_rad = GRIPPER_MAX_POS
        self.is_gripping = False
        self.gripping_started = False
        self.gripping_thread = None
        
    def connect(self):
        """Initialize and configure the gripper motor"""
        if not DM_AVAILABLE:
            self.last_message = "DM motor library not available"
            return False
        
        try:
            # Setup motor
            self.motor = Motor(DM_Motor_Type.DM4310, CONFIG["gripper_motor_id"], CONFIG["gripper_can_id"])
            
            # Setup serial connection
            self.serial_port = serial.Serial(CONFIG["gripper_port"], CONFIG["gripper_baud_rate"], timeout=0.5)
            self.motor_control = MotorControl(self.serial_port)
            
            # Add and enable motor
            self.motor_control.addMotor(self.motor)
            self.motor_control.enable(self.motor)
            
            # Set control mode
            self.motor_control.switchControlMode(self.motor, Control_Type.POS_VEL)
            
            self.connected = True
            self.last_message = "Gripper motor enabled successfully"
            print("✓ Gripper motor connected and enabled")
            return True
            
        except Exception as e:
            self.last_message = f"ERR: Gripper setup failed: {e}"
            print("❌ Gripper motor setup failed")
            return False
    
    def disconnect(self):
        """Disconnect and disable gripper motor"""
        if self.motor_control and self.motor:
            try:
                self.motor_control.disable(self.motor)
                print("✓ Gripper motor disabled")
            except:
                pass
        
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                print("✓ Gripper serial port closed")
            except:
                pass
        
        self.connected = False
    
    def move_to_percentage(self, percentage, velocity=None):
        """
        Move gripper to a specific closure percentage with adaptive gripping for closing operations.
        
        Args:
            percentage: Target closure percentage (0-100)
            velocity: Movement velocity (optional, uses default if not specified)
        """
        if not self.motor_control or not self.motor:
            self.last_message = "ERR: Gripper not available"
            return
        
        # Stop any existing adaptive gripping operation
        if self.is_gripping:
            self.stop_adaptive_gripping()
            time.sleep(0.1)  # Brief pause to ensure clean state
        
        try:
            # Convert percentage to radians
            target_position = percentage_to_position(percentage)
            safe_pos = safe_gripper_position(target_position)
            
            # Update state
            self.gripper_closure_percent = percentage
            self.gripper_position_rad = safe_pos
            
            if percentage > 0:
                # Always use adaptive gripping for closing operations
                self.last_message = f"Starting adaptive grip to {percentage:.1f}%"
                # Start adaptive gripping in a separate thread
                grip_thread = threading.Thread(
                    target=self._handle_adaptive_gripping, 
                    args=(percentage, velocity or ADAPTIVE_GRIPPING_CONFIG["closing_velocity"])
                )
                grip_thread.daemon = True
                self.gripping_thread = grip_thread
                grip_thread.start()
            else:
                # Direct movement for opening
                vel = velocity or ADAPTIVE_GRIPPING_CONFIG["opening_velocity"]
                self.motor_control.control_Pos_Vel(self.motor, safe_pos, vel)
                self.last_message = f"Moving to {percentage:.1f}% closed ({safe_pos:.3f} rad)"
            
        except Exception as e:
            self.last_message = f"ERR: Gripper move failed: {e}"
    
    def start_adaptive_gripping(self):
        """Start an adaptive gripping operation"""
        self.is_gripping = True
        self.gripping_started = True
        self.last_message = "Starting adaptive gripping..."
    
    def stop_adaptive_gripping(self):
        """Stop the current adaptive gripping operation"""
        self.is_gripping = False
        self.gripping_started = False
        self.gripping_thread = None
        self.last_message = "Adaptive gripping stopped"
    
    def _handle_adaptive_gripping(self, target_percentage, velocity):
        """
        Handle adaptive gripping by continuously monitoring sensor and stopping when object detected.
        This method runs in a separate thread.
        """
        if not self.motor_control or not self.motor:
            self.last_message = "ERR: Motor not available for adaptive gripping"
            return
        
        try:
            # Start adaptive gripping state
            self.start_adaptive_gripping()
            self.last_message = "Starting adaptive gripping..."
            
            # Get current position to use as hold position if needed
            try:
                current_pos = self.motor.getPosition()
            except:
                # Fallback to current state position if getPosition() fails
                current_pos = self.gripper_position_rad
            
            # Send command to move toward fully closed
            target_position = percentage_to_position(target_percentage)
            safe_pos = safe_gripper_position(target_position)
            self.motor_control.control_Pos_Vel(self.motor, safe_pos, velocity)
            
            # Update state
            self.gripper_closure_percent = target_percentage
            self.gripper_position_rad = safe_pos
            
            # Note: The actual gripping logic with sensor monitoring is handled in the main loop
            # This thread just initiates the movement and updates the state
            
        except Exception as e:
            self.last_message = f"ERR: Adaptive gripping failed: {e}"
            self.stop_adaptive_gripping()
    
    def check_gripping_condition(self, net_intensity, threshold):
        """
        Check if the net intensity exceeds the threshold during gripping.
        Returns True if object detected (should stop gripping).
        """
        if not self.is_gripping:
            return False
        
        return net_intensity > threshold
    
    def handle_object_detection(self):
        """Handle object detection during adaptive gripping"""
        if not self.is_gripping:
            return
        
        try:
            # Object detected - get current position and hold it
            try:
                hold_pos = self.motor.getPosition()
            except:
                # Fallback to current state position
                hold_pos = self.gripper_position_rad
            
            # Send command to hold current position with zero velocity to stop movement
            self.motor_control.control_Pos_Vel(self.motor, hold_pos, 0.0)
            
            # Send multiple stop commands to ensure motor stops
            for _ in range(3):
                self.motor_control.control_Pos_Vel(self.motor, hold_pos, 0.0)
                time.sleep(0.01)
            
            current_percentage = position_to_percentage(hold_pos)
            print(f"\n[STOP] Object detected! Holding at {current_percentage:.1f}% closed")
            
            # Update state to reflect the actual holding position
            self.gripper_position_rad = hold_pos
            self.gripper_closure_percent = current_percentage
            self.last_message = f"Object detected! Holding at {current_percentage:.1f}% closed"
            self.stop_adaptive_gripping()
            
        except Exception as e:
            self.last_message = f"ERR: Object detection handling failed: {e}"
            self.stop_adaptive_gripping()


class HardwareManager:
    """Main hardware manager that coordinates all motor systems"""
    
    def __init__(self):
        self.stepper = StepperMotorManager()
        self.gripper = GripperMotorManager()
        self.available = {
            'stepper': False,
            'gripper': DM_AVAILABLE
        }
    
    def initialize(self):
        """Initialize all hardware connections"""
        print("Initializing hardware connections...")
        
        # Initialize stepper motor
        self.available['stepper'] = self.stepper.connect()
        
        # Initialize gripper motor
        if self.available['gripper']:
            self.available['gripper'] = self.gripper.connect()
        
        return self.available
    
    def cleanup(self):
        """Clean up all hardware connections"""
        print("Cleaning up hardware connections...")
        
        # Release any pinched objects by opening gripper first
        if self.gripper.connected:
            try:
                print("Opening gripper to release objects...")
                self.gripper.move_to_percentage(0)  # Open to 0%
                time.sleep(1)  # Give time for gripper to open
            except:
                print("❌ Failed to open gripper")
        
        # Disconnect all hardware
        self.stepper.disconnect()
        self.gripper.disconnect()
    
    def get_status(self):
        """Get status of all hardware components"""
        return {
            'stepper': {
                'connected': self.stepper.connected,
                'last_message': self.stepper.last_message,
                'm1_pos': self.stepper.m1_target_pos,
                'm2_pos': self.stepper.m2_target_pos,
                'speed': self.stepper.speed,
                'current_angle': self.stepper.current_angle_deg
            },
            'gripper': {
                'connected': self.gripper.connected,
                'last_message': self.gripper.last_message,
                'closure_percent': self.gripper.gripper_closure_percent,
                'position_rad': self.gripper.gripper_position_rad,
                'is_gripping': self.gripper.is_gripping
            }
        }
