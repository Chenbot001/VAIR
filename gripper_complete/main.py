"""
Main application entry point for the Gripper Control System
Handles UI and the main loop, coordinates all modules
"""

import time
import threading
import signal
import sys
import os
import csv

from pynput import keyboard

from config import CONFIG, DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG, MANUAL_TILT_CONFIG
from hardware_manager import HardwareManager
from sensor_manager import SensorManager


class SystemState:
    """A class to hold the live state of the entire system"""
    def __init__(self):
        # Control mode state - always step-based
        self.object_diameter_mm = CONFIG["initial_diameter_mm"]
        
        # System state
        self.running = True
        
        # Hardware managers
        self.hardware = HardwareManager()
        self.sensors = SensorManager()
        
        # Thread references
        self.threads = []
    
    def update_diameter(self, diameter_mm):
        """Update diameter (control mode is always step-based)"""
        self.object_diameter_mm = diameter_mm


class GripperControlSystem:
    """Main control system class"""
    
    def __init__(self):
        self.state = SystemState()
        self.listener = None
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle interrupt signals for graceful shutdown"""
        print(f"\n\nReceived signal {signum}. Shutting down gracefully...")
        self.state.running = False
    
    def initialize(self):
        """Initialize all system components"""
        print("Initializing Complete Gripper Control System...")
        
        # Initialize hardware
        hardware_status = self.state.hardware.initialize()
        
        # Initialize sensors
        sensor_status = self.state.sensors.initialize()
        
        # Start background threads
        self._start_background_threads()
        
        # Initialize gripper to open position
        if self.state.hardware.gripper.connected:
            print("Initializing gripper to open position...")
            self.state.hardware.gripper.move_to_percentage(0)
            time.sleep(2)
        
        print("\n" + "="*60)
        print("           SYSTEM READY - STARTING CONTROL LOOP")
        print("="*60)
        print()
        
        return hardware_status and sensor_status
    
    def _start_background_threads(self):
        """Start all background threads"""
        # Start stepper reader thread
        if self.state.hardware.stepper.connected:
            stepper_thread = self.state.hardware.stepper.start_reader_thread(self.state)
            if stepper_thread:
                self.state.threads.append(stepper_thread)
                print("✓ Stepper reader thread started")
        
        # Start sensor reader thread
        if self.state.sensors.visuotactile.connected:
            sensor_thread = self.state.sensors.visuotactile.start_reader_thread(self.state)
            if sensor_thread:
                self.state.threads.append(sensor_thread)
                print("✓ Sensor reader thread started")
        
        # Give sensor reader time to start and get initial readings
        if self.state.sensors.visuotactile.connected:
            print("Waiting for sensor to stabilize...")
            time.sleep(1.0)
    
    def cleanup(self):
        """Clean up all system components"""
        print("\nExiting program...")
        self.state.running = False
        
        # Stop the keyboard listener first
        if self.listener:
            try:
                self.listener.stop()
                self.listener.join(timeout=2.0)
                print("✓ Keyboard listener stopped")
            except:
                pass
        
        # Clean up hardware and sensors
        self.state.hardware.cleanup()
        self.state.sensors.cleanup()
        
        # Force stop all background threads
        print("Stopping background threads...")
        active_threads = threading.enumerate()
        main_thread = threading.main_thread()
        
        for thread in active_threads:
            if thread != main_thread and thread.is_alive():
                try:
                    print(f"Waiting for thread: {thread.name}")
                    thread.join(timeout=2.0)
                    if thread.is_alive():
                        print(f"⚠ Thread {thread.name} did not stop gracefully")
                except Exception as e:
                    print(f"⚠ Error stopping thread {thread.name}: {e}")
        
        # Close any remaining matplotlib windows
        try:
            import matplotlib.pyplot as plt
            plt.close('all')
            print("✓ Matplotlib windows closed")
        except:
            pass
        
        # Close any remaining OpenCV windows
        try:
            import cv2
            cv2.destroyAllWindows()
            print("✓ OpenCV windows closed")
        except:
            pass
        
        print("✓ Cleanup completed")
        print("Exiting program...")
        
        # Force exit after cleanup
        os._exit(0)
    
    def update_display(self):
        """Update the system display"""
        # Clear screen using ANSI escape codes
        print('\033[2J\033[H', end='')
        sys.stdout.flush()
        
        print("=" * 60)
        print("         STEP-BASED GRIPPER CONTROL SYSTEM")
        print("=" * 60)
        
        # Get hardware status
        hw_status = self.state.hardware.get_status()
        sensor_status = self.state.sensors.get_status()
        
        # Stepper motor status
        print(f"ROTATION CONTROL (Step-Based):")
        print(f"  Motor Position: M1={hw_status['stepper']['m1_pos']:<4} | M2={hw_status['stepper']['m2_pos']:<4}")
        print(f"  Target Steps:   {CONFIG['fixed_target_steps']:>6}")
        print(f"  Object Diameter: {self.state.object_diameter_mm}mm")
        print(f"  Speed (steps/s): {CONFIG['fixed_motor_speed']:<4}")
        print(f"  Status: {'Connected' if hw_status['stepper']['connected'] else 'Disconnected'}")
        print(f"  Last Message: {hw_status['stepper']['last_message']}")
        
        print("-" * 60)
        
        # Gripper status
        gripper_status = "Connected" if hw_status['gripper']['connected'] else "Disconnected"
        gripper_closure = hw_status['gripper']['closure_percent']
        homing_allowed = gripper_closure <= 5.0  # Same threshold as in key handler
        homing_status = "✅ ALLOWED" if homing_allowed else "❌ BLOCKED"
        
        print("GRIPPER CONTROL (DM Motor):")
        print(f"  Closure: {gripper_closure:.1f}% | Position: {hw_status['gripper']['position_rad']:.3f} rad")
        print(f"  Homing Status: {homing_status} (requires ≤5% closure)")
        print(f"  Status: {gripper_status}")
        print(f"  Last Message: {hw_status['gripper']['last_message']}")
        
        print("-" * 60)
        
        # Sensor status
        sensor_available = "Connected" if sensor_status['visuotactile']['connected'] else "Disconnected"
        tilt_mode = "Manual" if MANUAL_TILT_CONFIG["enabled"] else "Auto"
        tilt_value = MANUAL_TILT_CONFIG["current_value"] if MANUAL_TILT_CONFIG["enabled"] else (
            sensor_status['visuotactile']['angle_offset'] if sensor_status['visuotactile']['angle_offset'] is not None else 0.0
        )
        print("DEPTH SENSOR:")
        print(f"  Max Intensity: {sensor_status['visuotactile']['max_intensity']:.3f}")
        print(f"  Baseline: {sensor_status['visuotactile']['baseline']:.3f} | Net: {sensor_status['visuotactile']['net_intensity']:.3f}")
        if sensor_status['visuotactile']['connected']:
            print(f"  Sensor FPS: {sensor_status['visuotactile']['sensor_fps']:.1f}")
        print(f"  Tilt Mode: {tilt_mode} | Angle: {tilt_value:.1f}°")
        print(f"  Status: {sensor_available}")
        
        print("-" * 60)
        
        # Adaptive gripping status
        gripping_status = "Active" if hw_status['gripper']['is_gripping'] else "Inactive"
        print("ADAPTIVE GRIPPING:")
        print(f"  Threshold: {ADAPTIVE_GRIPPING_CONFIG['threshold']:.3f}")
        if hw_status['gripper']['is_gripping']:
            net_intensity = sensor_status['visuotactile']['net_intensity']
            threshold = ADAPTIVE_GRIPPING_CONFIG['threshold']
            print(f"  Net Intensity: {net_intensity:.3f} / {threshold:.3f}")
            print(f"  Will Stop: {net_intensity > threshold}")
        
        print("-" * 60)
        
        # UR Robot status
        ur_status = "Connected" if hw_status['ur_robot']['connected'] else "Disconnected"
        print("UR ROBOT CONTROL:")
        if hw_status['ur_robot']['current_pose']:
            pose = hw_status['ur_robot']['current_pose']
            print(f"  Position: X={pose[0]:.3f} | Y={pose[1]:.3f} | Z={pose[2]:.3f}")
            print(f"  Rotation: RX={pose[3]:.3f} | RY={pose[4]:.3f} | RZ={pose[5]:.3f}")
        print(f"  Status: {ur_status}")
        print(f"  Last Message: {hw_status['ur_robot']['last_message']}")
        
        print("-" * 60)
        
        # Controls
        print("CONTROLS:")
        print("  UR Robot:  [Q/E] Up/Down | [A/D] Left/Right | [W/S] Forward/Backward")
        print("  Gripper Rotation: [J] CCW | [L] CW")
        print("  Diameter:  [1] 1mm | [2/3/4/5] 2-5mm")
        print("  Gripper:   [O] Open | [C] Close (Adaptive)")
        print("  Sensor:    [R] Calibrate Baseline")
        print("  Tilt:      [N] Decrease | [M] Increase | [T] Toggle Mode")
        print("  General:   [H] Home Motors* | [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit")
        print("             *Homing only works when gripper is fully open")
        print("=" * 60)
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            # --- UR Robot Controls ---
            if key.char == 'q':  # UR Robot Up
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('up')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 'e':  # UR Robot Down
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('down')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 'a':  # UR Robot Left
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('left')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 'd':  # UR Robot Right
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('right')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 'w':  # UR Robot Forward
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('forward')
                else:
                    print("❌ UR Robot not connected")
            elif key.char == 's':  # UR Robot Backward
                if self.state.hardware.ur_robot.connected:
                    self.state.hardware.ur_robot.move_step('backward')
                else:
                    print("❌ UR Robot not connected")

            # --- Stepper Motor Controls (Gripper Rotation) ---
            elif key.char == 'j':  # Rotate CCW
                # Set fixed motor speed
                self.state.hardware.stepper.speed = CONFIG["fixed_motor_speed"]
                
                print(f"🚀 Starting CCW step move: Target steps = {CONFIG['fixed_target_steps']}")
                
                # Step-based control for all diameters
                self.state.hardware.stepper.send_step_move_command(
                    CONFIG["fixed_target_steps"], 'ccw'
                )
                    
            elif key.char == 'l':  # Rotate CW
                # Set fixed motor speed
                self.state.hardware.stepper.speed = CONFIG["fixed_motor_speed"]
                
                print(f"🚀 Starting CW step move: Target steps = {CONFIG['fixed_target_steps']}")
                
                # Step-based control for all diameters
                self.state.hardware.stepper.send_step_move_command(
                    CONFIG["fixed_target_steps"], 'cw'
                )
            
            # --- Object Diameter Adjustment ---
            elif key.char == '1':
                self.state.update_diameter(1.0)
            elif key.char == '2':
                self.state.update_diameter(2)
            elif key.char == '3':
                self.state.update_diameter(3)
            elif key.char == '4':
                self.state.update_diameter(4)
            elif key.char == '5':
                self.state.update_diameter(5)

            # --- Gripper Controls ---
            elif key.char == 'o':  # Open gripper fully
                self.state.hardware.gripper.move_to_percentage(0)
            elif key.char == 'c':  # Close gripper (adaptive)
                if not self.state.hardware.gripper.is_gripping:
                    self.state.hardware.gripper.move_to_percentage(100)
                else:
                    self.state.hardware.gripper.last_message = "Gripping already in progress"

            # --- Sensor Controls ---
            elif key.char == 'r':  # Calibrate baseline intensity
                if self.state.sensors.visuotactile.sensor:
                    self.state.sensors.visuotactile.calibrate_baseline_intensity()
                else:
                    self.state.sensors.visuotactile.last_message = "ERR: Sensor not connected"

            # --- Tilt Controls ---
            elif key.char == 'n':  # Decrease manual tilt
                self.state.sensors.decrement_manual_tilt()
            elif key.char == 'm':  # Increase manual tilt
                self.state.sensors.increment_manual_tilt()
            elif key.char == 't':  # Toggle tilt mode
                self.state.sensors.toggle_manual_tilt_mode()

            # --- Utility Commands ---
            elif key.char == 'h':  # Home stepper motors (safety constraint: gripper must be fully open)
                gripper_closure = self.state.hardware.gripper.gripper_closure_percent
                if gripper_closure <= 5.0:  # Allow small tolerance for "fully open" (≤5%)
                    if self.state.hardware.stepper.connected:
                        print(f"🏠 Starting Python homing sequence...")
                        print(f"⚠ Please wait ~6 seconds for homing to complete...")
                        
                        # Perform Python-based homing sequence
                        homing_speed = CONFIG["homing_speed"]
                        
                        # Step 1: Move both motors to maximum position (1000)
                        print(f"   Step 1/3: Moving to max position (1000, 1000)")
                        self.state.hardware.stepper.send_move_command(1000, 1000, homing_speed)
                        time.sleep(1.5)  # Increased delay to ensure movement completes
                        
                        # Step 2: Move both motors to minimum position (0)
                        print(f"   Step 2/3: Moving to min position (0, 0)")
                        self.state.hardware.stepper.send_move_command(0, 0, homing_speed)
                        time.sleep(2.5)  # Increased delay to ensure movement completes
                        
                        # Step 3: Move both motors to center position (500, 500)
                        print(f"   Step 3/3: Moving to center position (500, 500)")
                        self.state.hardware.stepper.send_move_command(500, 500, homing_speed)
                        time.sleep(1.0)  # Allow time for final positioning
                        
                        # Reset angle tracking to zero
                        self.state.hardware.stepper.current_angle_deg = 0.0
                        
                        self.state.hardware.stepper.last_message = "Python homing sequence completed"
                        print(f"✅ Homing sequence completed - motors at center, angle reset to 0°")
                    else:
                        print(f"❌ Cannot home: Stepper motors not connected")
                else:
                    print(f"⚠ Motor homing BLOCKED: Gripper must be fully open (currently {gripper_closure:.1f}% closed)")
                    print(f"📋 Press [O] to open gripper first, then retry homing with [H]")
                    self.state.hardware.stepper.last_message = f"Homing blocked: gripper {gripper_closure:.1f}% closed"
            elif key.char == 'x':  # Reset stepper position to center and angle
                self.state.hardware.stepper.reset_to_center()

        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.space:
                # Stop all motors
                self.state.hardware.stepper.stop_motors()
                self.state.hardware.stepper.last_message = "STOP command sent"
                self.state.hardware.gripper.last_message = "Manual stop requested"
            elif key == keyboard.Key.esc:
                # Stop the listener and the program
                self.state.running = False
                return False
    
    def run(self):
        """Main application loop"""
        try:
            # Initialize system
            if not self.initialize():
                print("❌ System initialization failed")
                return
            
            # Small delay to ensure clean transition to control loop
            time.sleep(0.5)

            # Start the keyboard listener
            self.listener = keyboard.Listener(on_press=self.on_press)
            self.listener.start()
            
            last_update = 0
            while self.state.running:
                try:
                    current_time = time.time()
                    
                    # Check for adaptive gripping conditions
                    if (self.state.hardware.gripper.is_gripping and 
                        self.state.sensors.visuotactile.connected):
                        net_intensity = self.state.sensors.visuotactile.net_intensity
                        if self.state.sensors.check_gripping_condition(net_intensity):
                            self.state.hardware.gripper.handle_object_detection()
                    
                    # Update display periodically
                    if current_time - last_update >= DISPLAY_CONFIG["update_interval_seconds"]:
                        self.update_display()
                        last_update = current_time
                    
                    time.sleep(0.1)  # Faster polling for keyboard input
                    
                except KeyboardInterrupt:
                    print("\n\nKeyboard interrupt received. Shutting down...")
                    break
        
        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt received. Shutting down...")
        except Exception as e:
            print(f"\nAn unexpected error occurred: {e}")
        finally:
            self.cleanup()


def main():
    """Main entry point"""
    control_system = GripperControlSystem()
    control_system.run()


if __name__ == "__main__":
    main()
