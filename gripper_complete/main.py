"""
Main application entry point for the Gripper Control System
Handles UI and the main loop, coordinates all modules
"""

import time
import threading
import signal
import sys
import os
import json
import csv

from pynput import keyboard

from config import CONFIG, DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG, ACU_POSES
from hardware_manager import HardwareManager
from sensor_manager import SensorManager


class AcupunctureRecorder:
    """Manages recording during acupuncture experiment sessions"""
    
    def __init__(self, system_state):
        self.system_state = system_state
        self.recording = False
        self.session_start_time = None
        self.session_name = ""
        self.current_operation = "wait"
        self.operation_start_time = None
        self.fz_samples = []  # (timestamp, elapsed_time, fz_value, operation)
        self.last_snapshot_time = 0
        
        # Create data directories
        self.ensure_directories()
    
    def ensure_directories(self):
        """Create necessary data directories"""
        base_dir = os.path.join(os.path.dirname(__file__), "..", "data")
        acupuncture_dir = os.path.join(base_dir, "acupuncture")
        
        try:
            os.makedirs(base_dir, exist_ok=True)
            os.makedirs(acupuncture_dir, exist_ok=True)
        except Exception as e:
            print(f"❌ Error creating data directories: {e}")
    
    def start_session(self, session_name):
        """Start a new recording session"""
        if self.recording:
            self.stop_session()
        
        # Zero the Bota sensor first
        if self.system_state.sensors.bota.connected:
            self.system_state.sensors.bota.zero_fz()
            print(f"🔄 Zeroed Bota sensor for session: {session_name}")
        
        self.recording = True
        self.session_start_time = time.time()
        self.session_name = session_name
        self.current_operation = "wait"
        self.operation_start_time = self.session_start_time
        self.fz_samples = []
        self.last_snapshot_time = 0
        
        print(f"🔴 Started recording session: {session_name}")
    
    def stop_session(self):
        """Stop current recording session and save data"""
        if not self.recording:
            return
        
        print(f"⏹️ Stopping recording session: {self.session_name}")
        
        # Save force data to CSV
        self.save_force_csv()
        
        # Reset state
        self.recording = False
        self.session_start_time = None
        self.session_name = ""
        self.current_operation = "wait"
        self.operation_start_time = None
        self.fz_samples = []
        self.last_snapshot_time = 0
    
    def set_operation(self, operation):
        """Set current operation and reset operation timer"""
        self.current_operation = operation
        self.operation_start_time = time.time()
        print(f"   📝 Operation: {operation}")
    
    def update_recording(self):
        """Update recording with current sensor data (call this frequently during recording)"""
        if not self.recording or not self.system_state.sensors.bota.connected:
            return
        
        current_time = time.time()
        elapsed_session_time = current_time - self.session_start_time
        
        # Record Fz sample with operation
        fz_value = self.system_state.sensors.bota.current_fz
        self.fz_samples.append((current_time, elapsed_session_time, fz_value, self.current_operation))
        
        # Check if it's time for a snapshot (every 0.5 seconds)
        if elapsed_session_time - self.last_snapshot_time >= 0.5:
            self.take_snapshot(current_time, elapsed_session_time)
            self.last_snapshot_time = elapsed_session_time
    
    def take_snapshot(self, current_time, elapsed_session_time):
        """Take a sensor snapshot and save to JSON"""
        if not self.system_state.sensors.visuotactile.connected:
            return
        
        try:
            # Get snapshot from visuotactile sensor
            snapshot_data = self.system_state.sensors.visuotactile.capture_snapshot()
            
            if snapshot_data:
                # Calculate operation elapsed time
                operation_elapsed = current_time - self.operation_start_time if self.operation_start_time else 0
                
                # Add acupuncture-specific data
                enhanced_snapshot = snapshot_data.copy()
                enhanced_snapshot.update({
                    'session_elapsed_time': elapsed_session_time,
                    'operation': self.current_operation,
                    'operation_elapsed_time': operation_elapsed,
                    'bota_fz': self.system_state.sensors.bota.current_fz if self.system_state.sensors.bota.connected else 0.0
                })
                
                # Save to JSON file
                self.save_snapshot_json(enhanced_snapshot, current_time)
                
        except Exception as e:
            print(f"❌ Error taking snapshot: {e}")
    
    def save_snapshot_json(self, snapshot_data, timestamp):
        """Save snapshot data to JSON file"""
        try:
            acupuncture_dir = os.path.join(os.path.dirname(__file__), "..", "data", "acupuncture")
            
            # Create filename with operation and timestamp
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(timestamp))
            milliseconds = int((timestamp % 1) * 1000)
            filename = f"snapshot_{self.current_operation}_{timestamp_str}_{milliseconds:03d}.json"
            
            file_path = os.path.join(acupuncture_dir, filename)
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(snapshot_data, f, indent=2, default=str)
                
        except Exception as e:
            print(f"❌ Error saving snapshot JSON: {e}")
    
    def save_force_csv(self):
        """Save force data to CSV file"""
        if not self.fz_samples:
            return
        
        try:
            data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
            
            # Create filename with timestamp only
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.session_start_time))
            filename = f"acu_force_{timestamp_str}.csv"
            
            file_path = os.path.join(data_dir, filename)
            
            with open(file_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'elapsed_time_s', 'fz_value_N', 'operation'])
                
                for timestamp, elapsed_time, fz_value, operation in self.fz_samples:
                    writer.writerow([timestamp, f"{elapsed_time:.3f}", f"{fz_value:.6f}", operation])
                    
            print(f"✅ Force data saved: {file_path} ({len(self.fz_samples)} samples)")
            
        except Exception as e:
            print(f"❌ Error saving force CSV: {e}")


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
        
        # Acupuncture recorder
        self.acupuncture_recorder = None
    
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
        
        # Initialize gripper to default open position
        if self.state.hardware.gripper.connected:
            print(f"Initializing gripper to default open position ({CONFIG['gripper_default_open_percent']}% closed)...")
            self.state.hardware.gripper.move_to_percentage(CONFIG["gripper_default_open_percent"])
            time.sleep(2)
        
        # Initialize acupuncture recorder
        self.state.acupuncture_recorder = AcupunctureRecorder(self.state)
        
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
        
        # Start Bota force sensor reader thread
        if self.state.sensors.bota.connected:
            bota_thread = self.state.sensors.bota.start_reader_thread(self.state)
            if bota_thread:
                self.state.threads.append(bota_thread)
                print("✓ Bota force sensor reader thread started")
        
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
    
    def run_acupuncture_sequence(self):
        """Run the acupuncture experiment sequence in a separate thread"""
        if not self.state.hardware.ur_robot.connected:
            print("❌ Cannot run acupuncture sequence: UR Robot not connected")
            return
        
        def sequence_thread():
            try:
                print("🎯 Starting Acupuncture Experiment Sequence...")
                print("   This will take approximately 20 seconds to complete")
                
                # Step 1: Move to s_u
                print("   Step 1/6: Moving to s_u (shallow upper)")
                self.state.hardware.ur_robot.move_to_pose(ACU_POSES["s_u"])
                time.sleep(1)
                
                # Step 2: Move to s_d
                print("   Step 2/6: Moving to s_d (shallow deeper)")
                self.state.hardware.ur_robot.move_to_pose(ACU_POSES["s_d"])
                time.sleep(1)
                
                # Manipulation sequence between steps 2 and 3 (7-step sequence)
                print("   → Closing gripper")
                self.state.hardware.gripper.move_to_percentage(100)
                time.sleep(8)
                
                # START RECORDING SESSION 1 (shallow object)
                recorder = self.state.acupuncture_recorder
                recorder.start_session("left")
                
                # Get current pose and lift object
                current_pose = self.state.hardware.ur_robot.current_pose.copy()
                lift_pose = current_pose.copy()
                lift_pose[2] += 0.03  # Add 0.03m to Z coordinate
                
                print("   → Lifting object (+0.03m)")
                recorder.set_operation("lift")
                self.state.hardware.ur_robot.move_to_pose(lift_pose)
                self._recording_sleep(1, recorder)
                
                print("   → Lowering object back to position (-0.03m)")
                recorder.set_operation("lower")
                self.state.hardware.ur_robot.move_to_pose(current_pose)
                self._recording_sleep(2, recorder)
                
                print("   → Rotating 20 steps CW")
                recorder.set_operation("cw")
                self.state.hardware.stepper.send_step_move_command(20, 'cw')
                self._recording_sleep(1, recorder)
                
                print("   → Rotating 40 steps CCW")
                recorder.set_operation("ccw")
                self.state.hardware.stepper.send_step_move_command(40, 'ccw')
                self._recording_sleep(1, recorder)
                
                print("   → Rotating 20 steps CW (back to original)")
                recorder.set_operation("cw")
                self.state.hardware.stepper.send_step_move_command(20, 'cw')
                self._recording_sleep(1, recorder)
                
                # STOP RECORDING SESSION 1
                recorder.stop_session()
                
                print("   → Opening gripper")
                # Ensure adaptive gripping is stopped before opening
                if self.state.hardware.gripper.is_gripping:
                    print("   → Stopping adaptive gripping...")
                    self.state.hardware.gripper.stop_adaptive_gripping()
                    time.sleep(0.2)
                # Use opening velocity for faster gripper opening
                self.state.hardware.gripper.move_to_percentage(CONFIG["gripper_default_open_percent"], 
                                                             ADAPTIVE_GRIPPING_CONFIG["opening_velocity"])
                time.sleep(3)
                
                # Step 3: Move back to s_u
                print("   Step 3/6: Moving back to s_u (shallow upper)")
                self.state.hardware.ur_robot.move_to_pose(ACU_POSES["s_u"])
                time.sleep(1)
                
                # Step 4: Move to l_u
                print("   Step 4/6: Moving to l_u (lateral upper)")
                self.state.hardware.ur_robot.move_to_pose(ACU_POSES["l_u"])
                time.sleep(1)
                
                # Step 5: Move to l_d
                print("   Step 5/6: Moving to l_d (lateral deeper)")
                self.state.hardware.ur_robot.move_to_pose(ACU_POSES["l_d"])
                time.sleep(1)
                
                # Manipulation sequence between steps 5 and 6 (7-step sequence)
                print("   → Closing gripper")
                self.state.hardware.gripper.move_to_percentage(100)
                time.sleep(8)
                
                # START RECORDING SESSION 2 (lateral object)
                recorder.start_session("right")
                
                # Get current pose and lift object
                current_pose = self.state.hardware.ur_robot.current_pose.copy()
                lift_pose = current_pose.copy()
                lift_pose[2] += 0.03  # Add 0.03m to Z coordinate
                
                print("   → Lifting object (+0.03m)")
                recorder.set_operation("lift")
                self.state.hardware.ur_robot.move_to_pose(lift_pose)
                self._recording_sleep(1, recorder)
                
                print("   → Lowering object back to position (-0.03m)")
                recorder.set_operation("lower")
                self.state.hardware.ur_robot.move_to_pose(current_pose)
                self._recording_sleep(2, recorder)
                
                print("   → Rotating 20 steps CW")
                recorder.set_operation("cw")
                self.state.hardware.stepper.send_step_move_command(20, 'cw')
                self._recording_sleep(1, recorder)
                
                print("   → Rotating 40 steps CCW")
                recorder.set_operation("ccw")
                self.state.hardware.stepper.send_step_move_command(40, 'ccw')
                self._recording_sleep(1, recorder)
                
                print("   → Rotating 20 steps CW (back to original)")
                recorder.set_operation("cw")
                self.state.hardware.stepper.send_step_move_command(20, 'cw')
                self._recording_sleep(1, recorder)
                
                # STOP RECORDING SESSION 2
                recorder.stop_session()
                
                print("   → Opening gripper")
                # Ensure adaptive gripping is stopped before opening
                if self.state.hardware.gripper.is_gripping:
                    print("   → Stopping adaptive gripping...")
                    self.state.hardware.gripper.stop_adaptive_gripping()
                    time.sleep(0.2)
                # Use opening velocity for faster gripper opening
                self.state.hardware.gripper.move_to_percentage(CONFIG["gripper_default_open_percent"], 
                                                             ADAPTIVE_GRIPPING_CONFIG["opening_velocity"])
                time.sleep(3)
                
                # Step 6: Move back to l_u
                print("   Step 6/6: Moving back to l_u (lateral upper)")
                self.state.hardware.ur_robot.move_to_pose(ACU_POSES["l_u"])
                
                print("✅ Acupuncture Experiment Sequence Completed!")
                
            except Exception as e:
                print(f"❌ Acupuncture sequence failed: {e}")
        
        # Run sequence in background thread to not block main loop
        sequence = threading.Thread(target=sequence_thread, name="AcupunctureSequence", daemon=True)
        sequence.start()
        print("🚀 Acupuncture sequence started in background...")
    
    def _recording_sleep(self, duration, recorder):
        """Sleep while updating recording data"""
        end_time = time.time() + duration
        while time.time() < end_time:
            if recorder.recording:
                recorder.update_recording()
            time.sleep(0.05)  # Update recording every 50ms
    
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
        homing_allowed = gripper_closure <= CONFIG["gripper_default_open_percent"]  # Same threshold as in key handler
        homing_status = "✅ ALLOWED" if homing_allowed else "❌ BLOCKED"
        
        print("GRIPPER CONTROL (DM Motor):")
        print(f"  Closure: {gripper_closure:.1f}% | Position: {hw_status['gripper']['position_rad']:.3f} rad")
        print(f"  Homing Status: {homing_status} (requires ≤{CONFIG['gripper_default_open_percent']}% closure)")
        print(f"  Status: {gripper_status}")
        print(f"  Last Message: {hw_status['gripper']['last_message']}")
        
        print("-" * 60)
        
        # Sensor status
        sensor_available = "Connected" if sensor_status['visuotactile']['connected'] else "Disconnected"
        print("DEPTH SENSOR:")
        print(f"  Max Intensity: {sensor_status['visuotactile']['max_intensity']:.3f}")
        print(f"  Baseline: {sensor_status['visuotactile']['baseline']:.3f} | Net: {sensor_status['visuotactile']['net_intensity']:.3f}")
        if sensor_status['visuotactile']['connected']:
            print(f"  Sensor FPS: {sensor_status['visuotactile']['sensor_fps']:.1f}")
        print(f"  Status: {sensor_available}")
        
        print("-" * 60)
        
        # Bota Force Sensor status
        bota_status = sensor_status['bota']
        bota_connected = "Connected" if bota_status['connected'] else "Disconnected"
        print("BOTA FORCE SENSOR:")
        print(f"  Force Z (Fz): {bota_status['current_fz']:.4f} N | Offset: {bota_status['fz_offset']:.4f} N")
        if bota_status['latest_data'] and 'Mz' in bota_status['latest_data']:
            torque_z = bota_status['latest_data']['Mz']
            print(f"  Torque Z (Mz): {torque_z:.4f} Nm")
        else:
            print(f"  Torque Z (Mz): N/A")
        if bota_status['sampling_rate']:
            print(f"  Sampling Rate: {bota_status['sampling_rate']} Hz")
        else:
            print(f"  Sampling Rate: N/A")
        print(f"  Status: {bota_connected}")
        print(f"  Last Message: {bota_status['last_message']}")
        
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
        
        # Acupuncture recording status
        recorder = self.state.acupuncture_recorder
        if recorder:
            recording_status = "🔴 Recording" if recorder.recording else "⚪ Idle"
            print("ACUPUNCTURE RECORDING:")
            print(f"  Status: {recording_status}")
            if recorder.recording:
                elapsed = time.time() - recorder.session_start_time
                samples = len(recorder.fz_samples)
                print(f"  Session: {recorder.session_name} | Elapsed: {elapsed:.1f}s")
                print(f"  Operation: {recorder.current_operation} | Samples: {samples}")
        
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
        print("  Gripper:   [O] Open (50%) | [C] Close (Adaptive)")
        print("  Sensor:    [R] Calibrate Baseline | [Z] Zero Force Sensor")
        print("  Experiment:[P] Run Acupuncture Sequence (s_u→s_d→s_u→l_u→l_d→l_u)")
        print("  General:   [H] Home Motors* | [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit")
        print("             *Homing only works when gripper is at default open position (≤50% closed)")
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
            elif key.char == 'o':  # Open gripper to default position
                # Use opening velocity for faster gripper opening
                self.state.hardware.gripper.move_to_percentage(CONFIG["gripper_default_open_percent"], 
                                                             ADAPTIVE_GRIPPING_CONFIG["opening_velocity"])
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
            elif key.char == 'z':  # Zero Bota force sensor readings
                if self.state.sensors.bota.connected:
                    if self.state.sensors.bota.zero_fz():
                        print("✅ Bota force sensor readings zeroed")
                    else:
                        print("❌ Failed to zero Bota force sensor")
                else:
                    print("❌ Bota force sensor not connected")

            # --- Utility Commands ---
            elif key.char == 'h':  # Home stepper motors (safety constraint: gripper must be at default open position or more open)
                gripper_closure = self.state.hardware.gripper.gripper_closure_percent
                if gripper_closure <= CONFIG["gripper_default_open_percent"]:  # Allow homing when at default open position or more open
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
                    print(f"⚠ Motor homing BLOCKED: Gripper must be at default open position (≤{CONFIG['gripper_default_open_percent']}% closed, currently {gripper_closure:.1f}% closed)")
                    print(f"📋 Press [O] to open gripper first, then retry homing with [H]")
                    self.state.hardware.stepper.last_message = f"Homing blocked: gripper {gripper_closure:.1f}% closed"
            elif key.char == 'x':  # Reset stepper position to center and angle
                self.state.hardware.stepper.reset_to_center()
            elif key.char == 'p':  # Run acupuncture experiment sequence
                self.run_acupuncture_sequence()

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
                    
                    # Update acupuncture recording if active
                    if (self.state.acupuncture_recorder and 
                        self.state.acupuncture_recorder.recording):
                        self.state.acupuncture_recorder.update_recording()
                    
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
