"""
Main application entry point for the Gripper Control System
Handles UI and the main loop, coordinates all modules
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
        # Control mode state
        self.object_diameter_mm = CONFIG["initial_diameter_mm"]
        self.control_mode = self._determine_control_mode(self.object_diameter_mm)
        
        # Angle-based control state (for diameters 2-5mm)
        self.target_angle_deg = CONFIG["initial_target_angle_deg"]
        
        # Step-based control state (for diameters ≤1mm)
        self.target_steps = CONFIG["initial_target_steps"]
        
        # System state
        self.running = True
        
        # Hardware managers
        self.hardware = HardwareManager()
        self.sensors = SensorManager()
        
        # Thread references
        self.threads = []
    
    def _determine_control_mode(self, diameter_mm):
        """Determine control mode based on object diameter"""
        if diameter_mm <= 1.0:
            return "step"
        else:
            return "angle"
    
    def update_diameter_and_control_mode(self, diameter_mm):
        """Update diameter and automatically switch control mode"""
        self.object_diameter_mm = diameter_mm
        self.control_mode = self._determine_control_mode(diameter_mm)


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
        
        # Start encoder reader thread
        if self.state.sensors.encoder.connected:
            encoder_thread = self.state.sensors.encoder.start_reader_thread(self.state)
            if encoder_thread:
                self.state.threads.append(encoder_thread)
                print("✓ Encoder reader thread started")
        
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
        print("         ANGLE-BASED GRIPPER CONTROL SYSTEM")
        print("=" * 60)
        
        # Get hardware status
        hw_status = self.state.hardware.get_status()
        sensor_status = self.state.sensors.get_status()
        
        # Stepper motor status with angle information
        cw_steps_per_deg = get_steps_per_degree(self.state.object_diameter_mm, 'cw')
        ccw_steps_per_deg = get_steps_per_degree(self.state.object_diameter_mm, 'ccw')
        
        print(f"ROTATION CONTROL ({self.state.control_mode.upper()}-Based):")
        print(f"  Motor Position: M1={hw_status['stepper']['m1_pos']:<4} | M2={hw_status['stepper']['m2_pos']:<4}")
        print(f"  Current Angle:  {hw_status['stepper']['current_angle']:>6.1f}°")
        if self.state.control_mode == "angle":
            print(f"  Target Angle:   {self.state.target_angle_deg:>6.1f}°")
            print(f"  Steps/Degree:   CW={cw_steps_per_deg:.3f} | CCW={ccw_steps_per_deg:.3f}")
        else:
            print(f"  Target Steps:   {self.state.target_steps:>6}")
            print(f"  Step Mode:      {self.state.object_diameter_mm}mm (uncalibrated)")
        print(f"  Object Diameter: {self.state.object_diameter_mm}mm")
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
        
        # Encoder status
        encoder_status = "Connected" if sensor_status['encoder']['connected'] else "Disconnected"
        recording_status = "🔴 Recording" if sensor_status['encoder']['is_recording'] else "⚪ Idle"
        data_count = sensor_status['encoder']['data_count']
        
        print("ROTARY ENCODER:")
        print(f"  Status: {encoder_status}")
        print(f"  Recording: {recording_status} | Data Points: {data_count}")
        print(f"  Direction: CW (Fixed)")
        print(f"  Last Message: {sensor_status['encoder']['last_message']}")
        
        # Display latest saved CSV data for debugging
        latest_data = sensor_status['encoder']['latest_saved_data']
        if latest_data['initial_angle'] is not None:
            status_icon = "✅ SAVED" if latest_data['saved'] else "❌ DISCARDED"
            direction = latest_data.get('direction', 'Unknown')
            consecutive_count = latest_data.get('consecutive_count', 0)
            count_display = f" | Count: {consecutive_count}" if consecutive_count > 0 else ""
            print(f"  Last Operation: {direction.upper()} | Init={latest_data['initial_angle']}° | Target={latest_data['target_angle']}° | Measured={latest_data['measured_angle']}° | Error={latest_data['error']}° | {status_icon}{count_display}")
        else:
            print(f"  Last Operation: No data recorded yet")
        
        print("-" * 60)
        
        # Controls
        print("CONTROLS:")
        print("  Rotation:  [A] CCW | [D] CW | [W/S] Speed | [Q/E] Target ±5°/±50")
        print("  Diameter:  [0] 0.2mm | [H] 0.5mm | [1] 1mm | [2/3/4/5] 2-5mm")
        print("  Gripper:   [O] Open | [C] Close (Adaptive)")
        print("  Sensor:    [B] Calibrate Baseline")
        print("  Tilt:      [J] Decrease | [K] Increase | [T] Toggle Mode")
        print("  Encoder:   [Z] Zero")
        print("  General:   [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit")
        print("=" * 60)
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            # --- Stepper Motor Controls (Angle or Step based) ---
            if key.char == 'a':  # Rotate CCW
                if self.state.control_mode == "angle":
                    # Capture initial angle BEFORE sending move command to avoid race condition
                    initial_angle = self.state.hardware.stepper.get_current_angle_stable()
                    print(f"🚀 Starting CCW rotation: Initial angle = {initial_angle:.1f}°, Target = {self.state.target_angle_deg:.1f}°")
                    
                    # Angle-based control for diameters 2-5mm
                    self.state.hardware.stepper.send_angle_move_command(
                        self.state.target_angle_deg, 'ccw', self.state.object_diameter_mm
                    )
                    
                    # Start encoder recording for 5 seconds
                    if self.state.sensors.encoder.connected:
                        # Update recording metadata first
                        self.state.sensors.encoder.update_recording_metadata(
                            self.state.object_diameter_mm,
                            self.state.target_angle_deg,
                            self.state.sensors.visuotactile.max_depth_intensity,
                            self.state.sensors.visuotactile.get_rounded_tilt_angle()
                        )
                        # Start recording
                        self.state.sensors.encoder.start_encoder_recording(
                            'ccw', self.state.hardware.gripper.gripper_closure_percent, initial_angle
                        )
                else:
                    # Capture initial angle BEFORE sending move command to avoid race condition
                    initial_angle = self.state.hardware.stepper.get_current_angle_stable()
                    print(f"🚀 Starting CCW step move: Initial angle = {initial_angle:.1f}°, Target steps = {self.state.target_steps}")
                    
                    # Step-based control for diameters ≤1mm
                    self.state.hardware.stepper.send_step_move_command(
                        self.state.target_steps, 'ccw'
                    )
                    
                    # Start encoder recording for 5 seconds
                    if self.state.sensors.encoder.connected:
                        # Update recording metadata first (use steps as target)
                        self.state.sensors.encoder.update_recording_metadata(
                            self.state.object_diameter_mm,
                            self.state.target_steps,  # Use steps instead of angle
                            self.state.sensors.visuotactile.max_depth_intensity,
                            self.state.sensors.visuotactile.get_rounded_tilt_angle()
                        )
                        # Start recording
                        self.state.sensors.encoder.start_encoder_recording(
                            'ccw', self.state.hardware.gripper.gripper_closure_percent, initial_angle
                        )
                    
            elif key.char == 'd':  # Rotate CW
                if self.state.control_mode == "angle":
                    # Capture initial angle BEFORE sending move command to avoid race condition
                    initial_angle = self.state.hardware.stepper.get_current_angle_stable()
                    print(f"🚀 Starting CW rotation: Initial angle = {initial_angle:.1f}°, Target = {self.state.target_angle_deg:.1f}°")
                    
                    # Angle-based control for diameters 2-5mm
                    self.state.hardware.stepper.send_angle_move_command(
                        self.state.target_angle_deg, 'cw', self.state.object_diameter_mm
                    )
                    
                    # Start encoder recording for 5 seconds
                    if self.state.sensors.encoder.connected:
                        # Update recording metadata first
                        self.state.sensors.encoder.update_recording_metadata(
                            self.state.object_diameter_mm,
                            self.state.target_angle_deg,
                            self.state.sensors.visuotactile.max_depth_intensity,
                            self.state.sensors.visuotactile.get_rounded_tilt_angle()
                        )
                        # Start recording
                        self.state.sensors.encoder.start_encoder_recording(
                            'cw', self.state.hardware.gripper.gripper_closure_percent, initial_angle
                        )
                else:
                    # Capture initial angle BEFORE sending move command to avoid race condition
                    initial_angle = self.state.hardware.stepper.get_current_angle_stable()
                    print(f"🚀 Starting CW step move: Initial angle = {initial_angle:.1f}°, Target steps = {self.state.target_steps}")
                    
                    # Step-based control for diameters ≤1mm
                    self.state.hardware.stepper.send_step_move_command(
                        self.state.target_steps, 'cw'
                    )
                    
                    # Start encoder recording for 5 secondscx
                    if self.state.sensors.encoder.connected:
                        # Update recording metadata first (use steps as target)
                        self.state.sensors.encoder.update_recording_metadata(
                            self.state.object_diameter_mm,
                            self.state.target_steps,  # Use steps instead of angle
                            self.state.sensors.visuotactile.max_depth_intensity,
                            self.state.sensors.visuotactile.get_rounded_tilt_angle()
                        )
                        # Start recording
                        self.state.sensors.encoder.start_encoder_recording(
                            'cw', self.state.hardware.gripper.gripper_closure_percent, initial_angle
                        )
            
            # --- Speed Control ---
            elif key.char == 'w':
                self.state.hardware.stepper.speed += 50
            elif key.char == 's':
                self.state.hardware.stepper.speed = max(50, self.state.hardware.stepper.speed - 50)
                
            # --- Target Adjustment (Angle or Steps) ---
            elif key.char == 'e':
                if self.state.control_mode == "angle":
                    self.state.target_angle_deg += CONFIG["angle_increment_deg"]
                    self.state.target_angle_deg = min(self.state.target_angle_deg, CONFIG["max_angle_deg"])
                    # Reset counter when target angle changes
                    if self.state.sensors.encoder.connected:
                        self.state.sensors.encoder.reset_consecutive_saves_counter()
                else:
                    self.state.target_steps += CONFIG["step_increment"]
                    self.state.target_steps = min(self.state.target_steps, CONFIG["max_steps_movement"])
            elif key.char == 'q':
                if self.state.control_mode == "angle":
                    self.state.target_angle_deg = max(CONFIG["angle_increment_deg"], 
                                                   self.state.target_angle_deg - CONFIG["angle_increment_deg"])
                    # Reset counter when target angle changes
                    if self.state.sensors.encoder.connected:
                        self.state.sensors.encoder.reset_consecutive_saves_counter()
                else:
                    self.state.target_steps = max(CONFIG["step_increment"], 
                                               self.state.target_steps - CONFIG["step_increment"])
            
            # --- Object Diameter Adjustment ---
            elif key.char == '0':
                self.state.update_diameter_and_control_mode(0.2)
                # Reset counter when diameter changes
                if self.state.sensors.encoder.connected:
                    self.state.sensors.encoder.reset_consecutive_saves_counter()
            elif key.char == 'h':
                self.state.update_diameter_and_control_mode(0.5)  # 'h' for half (0.5mm)
                # Reset counter when diameter changes
                if self.state.sensors.encoder.connected:
                    self.state.sensors.encoder.reset_consecutive_saves_counter()
            elif key.char == '1':
                self.state.update_diameter_and_control_mode(1.0)
                # Reset counter when diameter changes
                if self.state.sensors.encoder.connected:
                    self.state.sensors.encoder.reset_consecutive_saves_counter()
            elif key.char == '2':
                self.state.update_diameter_and_control_mode(2)
                # Reset counter when diameter changes
                if self.state.sensors.encoder.connected:
                    self.state.sensors.encoder.reset_consecutive_saves_counter()
            elif key.char == '3':
                self.state.update_diameter_and_control_mode(3)
                # Reset counter when diameter changes
                if self.state.sensors.encoder.connected:
                    self.state.sensors.encoder.reset_consecutive_saves_counter()
            elif key.char == '4':
                self.state.update_diameter_and_control_mode(4)
                # Reset counter when diameter changes
                if self.state.sensors.encoder.connected:
                    self.state.sensors.encoder.reset_consecutive_saves_counter()
            elif key.char == '5':
                self.state.update_diameter_and_control_mode(5)
                # Reset counter when diameter changes
                if self.state.sensors.encoder.connected:
                    self.state.sensors.encoder.reset_consecutive_saves_counter()

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

            # --- Encoder Controls ---
            elif key.char == 'z':  # Zero encoder
                if self.state.sensors.encoder.instrument and self.state.sensors.encoder.connected:
                    self.state.sensors.encoder.zero_encoder()

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
    """Main entry point"""
    control_system = GripperControlSystem()
    control_system.run()


if __name__ == "__main__":
    main()
