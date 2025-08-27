"""
Main application entry point for the Thin Wire Gripper Control System
Handles UI and the main loop, coordinates all modules
Step-based control only for thin wires (0.2mm, 0.5mm, 1.0mm)
"""

import time
import threading
import signal
import sys
import os

from pynput import keyboard

from config import CONFIG, DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG, MANUAL_TILT_CONFIG
from hardware_manager import HardwareManager
from sensor_manager import SensorManager
from utils import get_steps_per_degree


class SystemState:
    """A class to hold the live state of the entire system"""
    def __init__(self):
        # Thin wire diameter options (0.2mm, 0.5mm, 1.0mm)
        self.object_diameter_mm = 0.5  # Default to 0.5mm
        self.control_mode = "step"  # Always step-based for thin wires
        
        # Step-based control parameters
        self.target_steps = 10  # Default 10 steps
        self.min_steps = 5      # Minimum 5 steps
        self.step_increment = 5 # Adjust in 5-step increments
        
        # System state
        self.running = True
        
        # Hardware managers (no encoder for thin wires)
        self.hardware = HardwareManager()
        self.sensors = SensorManager()
        
        # Thread references
        self.threads = []
    
    def update_diameter(self, diameter_mm):
        """Update diameter for thin wire operations"""
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
        print("Initializing Thin Wire Gripper Control System...")
        
        # Initialize hardware
        hardware_status = self.state.hardware.initialize()
        
        # Initialize sensors (no encoder for thin wires)
        sensor_status = self.state.sensors.initialize()
        
        # Start background threads
        self._start_background_threads()
        
        # Initialize gripper to open position
        if self.state.hardware.gripper.connected:
            print("Initializing gripper to open position...")
            self.state.hardware.gripper.move_to_percentage(0)
            time.sleep(2)
        
        print("\n" + "="*60)
        print("      THIN WIRE SYSTEM READY - STARTING CONTROL LOOP")
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
        
        # No encoder for thin wires - insufficient torsion
        
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
        print("         THIN WIRE GRIPPER CONTROL SYSTEM")
        print("=" * 60)
        
        # Get hardware status
        hw_status = self.state.hardware.get_status()
        sensor_status = self.state.sensors.get_status()
        
        print(f"ROTATION CONTROL (STEP-Based Only):")
        print(f"  Motor Position: M1={hw_status['stepper']['m1_pos']:<4} | M2={hw_status['stepper']['m2_pos']:<4}")
        print(f"  Current Angle:  {hw_status['stepper']['current_angle']:>6.1f}°")
        print(f"  Target Steps:   {self.state.target_steps:>6}")
        print(f"  Wire Diameter:  {self.state.object_diameter_mm}mm")
        print(f"  Speed (steps/s): {hw_status['stepper']['speed']:<4}")
        print(f"  Status: {'Connected' if hw_status['stepper']['connected'] else 'Disconnected'}")
        print(f"  Last Message: {hw_status['stepper']['last_message']}")
        
        print("-" * 60)
        
        # Gripper status
        gripper_status = "Connected" if hw_status['gripper']['connected'] else "Disconnected"
        print("GRIPPER CONTROL (DM Motor):")
        print(f"  Closure: {hw_status['gripper']['closure_percent']:.1f}% | Position: {hw_status['gripper']['position_rad']:.3f} rad")
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
        
        # Controls
        print("CONTROLS:")
        print("  Rotation:  [A] CCW | [D] CW | [W/S] Speed | [Q/E] Target ±5 steps")
        print("  Diameter:  [1] 0.2mm | [2] 0.5mm | [3] 1.0mm")
        print("  Gripper:   [O] Open | [C] Close (Adaptive)")
        print("  Sensor:    [B] Calibrate Baseline")
        print("  Tilt:      [J] Decrease | [K] Increase | [T] Toggle Mode")
        print("  General:   [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit")
        print("=" * 60)
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            # --- Stepper Motor Controls (Step-based only) ---
            if key.char == 'a':  # Rotate CCW
                print(f"🚀 Starting CCW step move: Target steps = {self.state.target_steps}")
                
                # Step-based control for thin wires
                self.state.hardware.stepper.send_step_move_command(
                    self.state.target_steps, 'ccw'
                )
                    
            elif key.char == 'd':  # Rotate CW
                print(f"🚀 Starting CW step move: Target steps = {self.state.target_steps}")
                
                # Step-based control for thin wires
                self.state.hardware.stepper.send_step_move_command(
                    self.state.target_steps, 'cw'
                )
            
            # --- Speed Control ---
            elif key.char == 'w':
                self.state.hardware.stepper.speed += 10
            elif key.char == 's':
                self.state.hardware.stepper.speed = max(10, self.state.hardware.stepper.speed - 10)
                
            # --- Target Step Adjustment ---
            elif key.char == 'e':
                self.state.target_steps += self.state.step_increment
                # No maximum limit for thin wire steps
            elif key.char == 'q':
                self.state.target_steps = max(self.state.min_steps, 
                                           self.state.target_steps - self.state.step_increment)
            
            # --- Wire Diameter Adjustment ---
            elif key.char == '1':
                self.state.update_diameter(0.2)
            elif key.char == '2':
                self.state.update_diameter(0.5)
            elif key.char == '3':
                self.state.update_diameter(1.0)

            # --- Gripper Controls ---
            elif key.char == 'o':  # Open gripper fully
                self.state.hardware.gripper.move_to_percentage(0)
            elif key.char == 'c':  # Close gripper (adaptive)
                if not self.state.hardware.gripper.is_gripping:
                    self.state.hardware.gripper.move_to_percentage(100)
                else:
                    self.state.hardware.gripper.last_message = "Gripping already in progress"

            # --- Sensor Controls ---
            elif key.char == 'b':  # Calibrate baseline intensity
                if self.state.sensors.visuotactile.sensor:
                    self.state.sensors.visuotactile.calibrate_baseline_intensity()
                else:
                    self.state.sensors.visuotactile.last_message = "ERR: Sensor not connected"

            # --- Tilt Controls ---
            elif key.char == 'j':  # Decrease manual tilt
                self.state.sensors.decrement_manual_tilt()
            elif key.char == 'k':  # Increase manual tilt
                self.state.sensors.increment_manual_tilt()
            elif key.char == 't':  # Toggle tilt mode
                self.state.sensors.toggle_manual_tilt_mode()

            # --- Utility Commands ---
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
    """Main entry point for thin wire control"""
    control_system = GripperControlSystem()
    control_system.run()


if __name__ == "__main__":
    main()
