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
import cv2
import numpy as np
import math
import csv
from datetime import datetime

from pynput import keyboard

# Import for finding hardware devices on Windows
try:
    import wmi
except ImportError:
    print("Warning: The 'wmi' library is required for camera detection.")
    print("Camera features will be disabled. Install with: pip install wmi")
    wmi = None

from config import CONFIG, DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG, MANUAL_TILT_CONFIG
from hardware_manager import HardwareManager
from sensor_manager import SensorManager
from utils import get_steps_per_degree

# --- Camera Configuration ---
# Unique part of your webcam's Device Instance Path:
TARGET_SERIAL_ID = "2B9D0D7D&0&0000"

# Stabilization parameters (always enabled)
ANGLE_SMOOTHING_FACTOR = 0.8  # Higher = more smoothing (0.0-1.0)
MIN_ANGLE_CHANGE = 0.5  # Minimum angle change to register (degrees)
ANGLE_HISTORY_SIZE = 5  # Number of recent angles to consider for median filtering


class AngleStabilizer:
    """Class to stabilize angle readings and reduce flickering"""
    def __init__(self, smoothing_factor=0.8, min_change=0.5, history_size=5):
        self.smoothing_factor = smoothing_factor
        self.min_change = min_change
        self.history_size = history_size
        self.previous_angle = None
        self.smoothed_angle = None
        self.angle_history = []
        
    def update(self, new_angle):
        """Update with new angle and return stabilized angle"""
        if new_angle is None:
            return self.smoothed_angle
            
        # Add to history for median filtering
        self.angle_history.append(new_angle)
        if len(self.angle_history) > self.history_size:
            self.angle_history.pop(0)
            
        # Use median of recent angles to reduce noise
        if len(self.angle_history) >= 3:
            # Handle angle wraparound for median calculation
            angles_unwrapped = self._unwrap_angles(self.angle_history)
            median_angle = np.median(angles_unwrapped)
            median_angle = median_angle % 360  # Wrap back to 0-360
        else:
            median_angle = new_angle
            
        # Initialize if first reading
        if self.smoothed_angle is None:
            self.smoothed_angle = median_angle
            self.previous_angle = median_angle
            return self.smoothed_angle
            
        # Calculate angle difference considering wraparound
        diff = self._angle_difference(median_angle, self.smoothed_angle)
        
        # Only update if change is significant enough
        if abs(diff) > self.min_change:
            # Apply exponential smoothing
            self.smoothed_angle = self._angle_lerp(self.smoothed_angle, median_angle, 1 - self.smoothing_factor)
            self.previous_angle = self.smoothed_angle
            
        return self.smoothed_angle
    
    def _angle_difference(self, angle1, angle2):
        """Calculate the shortest angular difference between two angles"""
        diff = angle1 - angle2
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff
    
    def _angle_lerp(self, angle1, angle2, t):
        """Linear interpolation between two angles considering wraparound"""
        diff = self._angle_difference(angle2, angle1)
        result = angle1 + diff * t
        return result % 360
    
    def _unwrap_angles(self, angles):
        """Unwrap angles to handle 0/360 boundary for median calculation"""
        if not angles:
            return angles
            
        unwrapped = [angles[0]]
        for i in range(1, len(angles)):
            diff = self._angle_difference(angles[i], unwrapped[-1])
            unwrapped.append(unwrapped[-1] + diff)
        return unwrapped


class CameraDataCollector:
    """Class to collect camera-based angle data during thin wire manipulation"""
    def __init__(self):
        self.is_recording = False
        self.recording_data = []
        self.recording_metadata = {}
        self.recording_start_time = None
        self.recording_thread = None
        self.initial_angle = None
        self.final_angle = None
        self.csv_file_path = os.path.join("gripper_complete", "encoder_data_steps.csv")
        
        # Counter for consecutive saves with same diameter+target_steps combination
        self.consecutive_saves_counter = 0
        self.last_combination = {'diameter_mm': None, 'target_steps': None}
        
        # Latest saved data for display
        self.latest_saved_data = {
            'diameter_mm': None,
            'target_steps': None,
            'speed': None,
            'direction': None,
            'angle': None,
            'saved': False,
            'consecutive_count': 0
        }
        
    def start_recording(self, direction, diameter_mm, target_steps, speed, initial_angle):
        """Start a 5-second recording session"""
        if self.is_recording:
            print("⚠ Recording already in progress")
            return
            
        self.is_recording = True
        self.recording_data = []
        self.recording_start_time = time.time()
        self.initial_angle = initial_angle
        self.final_angle = None
        
        # Store metadata
        self.recording_metadata = {
            'direction': direction,
            'diameter_mm': diameter_mm,
            'target_steps': target_steps,
            'speed': speed,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        
        print(f"🎥 Started camera recording: {direction.upper()} | {target_steps} steps | {diameter_mm}mm")
        
        # Start recording thread
        self.recording_thread = threading.Thread(target=self._recording_worker, daemon=True)
        self.recording_thread.start()
    
    def _recording_worker(self):
        """Background worker for 5-second recording"""
        try:
            duration = 5.0  # 5 seconds
            while self.is_recording and (time.time() - self.recording_start_time) < duration:
                time.sleep(0.1)  # Check every 100ms
            
            # Stop recording after 5 seconds
            self.stop_recording()
            
        except Exception as e:
            print(f"❌ Recording worker error: {e}")
            self.is_recording = False
    
    def update_angle(self, current_angle):
        """Update the current angle during recording"""
        if self.is_recording and current_angle is not None:
            elapsed_time = time.time() - self.recording_start_time
            self.recording_data.append({
                'timestamp': time.time(),
                'elapsed_time': elapsed_time,
                'angle': current_angle
            })
            self.final_angle = current_angle  # Keep updating final angle
    
    def stop_recording(self):
        """Stop recording and process data"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        
        # Calculate angular displacement with noise filtering
        angular_displacement = self._calculate_angular_displacement()
        
        if angular_displacement is not None:
            # Save to CSV
            self._save_to_csv(angular_displacement)
            print(f"✅ Recording completed: Angular displacement = {angular_displacement:.2f}°")
        else:
            print("❌ Recording failed: Could not calculate reliable angular displacement")
    
    def _calculate_angular_displacement(self):
        """Calculate angular displacement with noise filtering"""
        if not self.recording_data or len(self.recording_data) < 10:
            print("⚠ Insufficient data points for reliable measurement")
            return None
            
        # Extract angles from recording data
        angles = [data['angle'] for data in self.recording_data]
        
        # Use the initial angle captured at key press (no averaging since motor starts immediately)
        start_angle = self.initial_angle
        
        # Use robust end angle estimation from last 10 readings to reduce noise
        end_angles = angles[-10:]   # Last 10 readings (last second)
        end_angle_median = np.median(end_angles)
        
        # Calculate angular displacement considering 0°/360° wraparound
        displacement = self._angle_difference(end_angle_median, start_angle)
        
        return displacement
    
    def _angle_difference(self, angle2, angle1):
        """Calculate angular difference considering wraparound"""
        diff = angle2 - angle1
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff
    
    def reset_consecutive_saves_counter(self):
        """Reset the consecutive saves counter when parameters change"""
        self.consecutive_saves_counter = 0
        print(f"🔄 Data collection counter reset")
    
    def _save_to_csv(self, angular_displacement):
        """Save recording data to CSV file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
            
            # Check if file exists to determine if we need headers
            file_exists = os.path.exists(self.csv_file_path)
            
            # Ensure angular displacement is strictly non-negative
            abs_angular_displacement = abs(angular_displacement)
            
            # Check if diameter+target_steps combination has changed
            current_combination = {
                'diameter_mm': self.recording_metadata['diameter_mm'],
                'target_steps': self.recording_metadata['target_steps']
            }
            
            if (self.last_combination['diameter_mm'] != current_combination['diameter_mm'] or 
                self.last_combination['target_steps'] != current_combination['target_steps']):
                # Reset counter for new combination
                self.consecutive_saves_counter = 0
                self.last_combination = current_combination.copy()
            
            # Increment counter for this save
            self.consecutive_saves_counter += 1
            
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                fieldnames = ['diameter', 'speed', 'step', 'angle', 'direction']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write header if file is new
                if not file_exists:
                    writer.writeheader()
                
                # Write data row with consistent field names and order
                writer.writerow({
                    'diameter': self.recording_metadata['diameter_mm'],
                    'speed': self.recording_metadata['speed'], 
                    'step': self.recording_metadata['target_steps'],
                    'angle': f"{abs_angular_displacement:.2f}",  
                    'direction': self.recording_metadata['direction']
                })
                
                # Update latest saved data for display
                self.latest_saved_data = {
                    'diameter_mm': self.recording_metadata['diameter_mm'],
                    'target_steps': self.recording_metadata['target_steps'],
                    'speed': self.recording_metadata['speed'],
                    'direction': self.recording_metadata['direction'],
                    'angle': abs_angular_displacement,
                    'saved': True,
                    'consecutive_count': self.consecutive_saves_counter
                }
                
                print(f"💾 Data saved to CSV: {self.recording_metadata['diameter_mm']}mm | {self.recording_metadata['target_steps']} steps | {abs_angular_displacement:.2f}° | Count: {self.consecutive_saves_counter}")
                
        except Exception as e:
            print(f"❌ Error saving to CSV: {e}")
            # Update latest saved data to show failure
            self.latest_saved_data['saved'] = False
    
    def get_status(self):
        """Get current recording status"""
        return {
            'is_recording': self.is_recording,
            'data_points': len(self.recording_data),
            'elapsed_time': time.time() - self.recording_start_time if self.is_recording else 0
        }


def find_camera_index_by_serial(target_id):
    """Uses WMI to find the OpenCV index of a camera by its unique serial ID."""
    if wmi is None:
        return -1
        
    try:
        c = wmi.WMI()
        wmi_devices = [dev for dev in c.Win32_PnPEntity(PNPClass="Camera") if "USB" in dev.DeviceID]

        if not wmi_devices:
            return -1

        for index, dev in enumerate(wmi_devices):
            if target_id.lower() in dev.DeviceID.lower():
                return index
                
        return -1
    except:
        return -1


def get_pointer_angle_and_line(frame):
    """Angle detection with 0-360° range, 0° pointing up, clockwise direction."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply stronger Gaussian blur to reduce noise and specs
    blurred = cv2.GaussianBlur(gray, (7, 7), 1.5)
    
    # Adaptive thresholding with optimized parameters to reduce noise
    adaptive_thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                          cv2.THRESH_BINARY_INV, 15, 4)
    
    # Apply circular region of interest mask
    h, w = adaptive_thresh.shape
    center = (w//2, h//2)
    radius = min(w, h) // 4  # Adjust radius as needed
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    
    # Apply mask to threshold image
    thresh = cv2.bitwise_and(adaptive_thresh, mask)
    
    # Apply morphological operations to remove specs and clean up noise
    kernel_small = np.ones((2, 2), np.uint8)  # Smaller kernel for fine noise
    kernel_large = np.ones((3, 3), np.uint8)  # Larger kernel for general cleanup
    
    # Remove small noise specs
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel_small, iterations=2)
    # Fill small gaps
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_large, iterations=1)
    
    # Remove connected components smaller than threshold to eliminate tiny specs
    def remove_small_components(img, min_size=15):
        """Remove small connected components (specs) from binary image"""
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)
        # Create output image
        cleaned = np.zeros_like(img)
        # Keep components larger than min_size (skip background label 0)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_size:
                cleaned[labels == i] = 255
        return cleaned
    
    # Apply component filtering to remove tiny specs
    thresh = remove_small_components(thresh, min_size=15)

    # Use more restrictive parameters for line detection to avoid noise
    lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, threshold=30, minLineLength=35, maxLineGap=2)
    
    if lines is not None:
        # Filter lines by length and proximity to center
        valid_lines = []
        center_x, center_y = center
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate line length
            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # Calculate minimum distance from line to center
            # Distance from point to line formula
            A = y2 - y1
            B = x1 - x2
            C = x2 * y1 - x1 * y2
            if A*A + B*B > 0:  # Avoid division by zero
                dist_to_center = abs(A * center_x + B * center_y + C) / math.sqrt(A*A + B*B)
            else:
                continue
            
            # Only keep lines that are long enough and pass near the center
            if length > 30 and dist_to_center < 15:  # Stricter thresholds for stability
                valid_lines.append((line, length))
        
        if valid_lines:
            # Get the longest valid line
            longest_line, _ = max(valid_lines, key=lambda x: x[1])
            x1, y1, x2, y2 = longest_line[0]
            
            # Calculate distances from both endpoints to center
            center_x, center_y = center
            dist1 = math.sqrt((x1 - center_x)**2 + (y1 - center_y)**2)
            dist2 = math.sqrt((x2 - center_x)**2 + (y2 - center_y)**2)
            
            # Use the point closest to center as start point
            if dist1 <= dist2:
                start_x, start_y = x1, y1
                end_x, end_y = x2, y2
            else:
                start_x, start_y = x2, y2
                end_x, end_y = x1, y1
            
            # Calculate vector from start to end
            dx = end_x - start_x
            dy = end_y - start_y
            
            # Calculate angle: atan2 gives -π to π, we want 0-360° with 0° pointing up
            angle_rad = math.atan2(dx, -dy)  # Note: swapped dx, dy and negated dy for "up" = 0°
            
            # Convert to 0-360° range
            angle_deg = math.degrees(angle_rad)
            if angle_deg < 0:
                angle_deg += 360
                
            return angle_deg, (start_x, start_y, end_x, end_y), thresh, center
    
    return None, None, thresh, (w//2, h//2)


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
        
        # Camera and angle detection
        self.camera = None
        self.angle_stabilizer = None
        self.current_angle = None
        self.camera_enabled = False
        self.show_camera_windows = True  # Enabled by default for better visualization
        
        # Data collection for thin wire recordings
        self.data_collector = CameraDataCollector()
        
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
        
        # Initialize camera for angle detection
        self._initialize_camera()
        
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
    
    def _initialize_camera(self):
        """Initialize camera for pointer angle detection"""
        print("Initializing camera for angle detection...")
        
        try:
            # Find camera by serial ID
            cam_index = find_camera_index_by_serial(TARGET_SERIAL_ID)
            if cam_index != -1:
                self.state.camera = cv2.VideoCapture(cam_index)
                if self.state.camera.isOpened():
                    # Initialize angle stabilizer
                    self.state.angle_stabilizer = AngleStabilizer(
                        smoothing_factor=ANGLE_SMOOTHING_FACTOR,
                        min_change=MIN_ANGLE_CHANGE,
                        history_size=ANGLE_HISTORY_SIZE
                    )
                    self.state.camera_enabled = True
                    print(f"✓ Camera initialized successfully at index {cam_index}")
                else:
                    print(f"❌ Could not open camera at index {cam_index}")
                    self.state.camera_enabled = False
            else:
                print(f"❌ Camera with serial ID '{TARGET_SERIAL_ID}' not found")
                self.state.camera_enabled = False
        except Exception as e:
            print(f"❌ Camera initialization failed: {e}")
            self.state.camera_enabled = False
    
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
        
        # Start camera processing thread
        if self.state.camera_enabled:
            camera_thread = threading.Thread(target=self._camera_processing_thread, daemon=True)
            camera_thread.start()
            self.state.threads.append(camera_thread)
            print("✓ Camera processing thread started")
        
        # No encoder for thin wires - insufficient torsion
        
        # Give sensor reader time to start and get initial readings
        if self.state.sensors.visuotactile.connected:
            print("Waiting for sensor to stabilize...")
            time.sleep(1.0)
    
    def _camera_processing_thread(self):
        """Background thread for camera processing and angle detection"""
        while self.state.running and self.state.camera_enabled:
            try:
                ret, frame = self.state.camera.read()
                if not ret:
                    continue
                
                # Get pointer angle and line coordinates
                raw_angle, line_coords, thresh, center = get_pointer_angle_and_line(frame)
                
                # Apply stabilization
                stabilized_angle = self.state.angle_stabilizer.update(raw_angle)
                self.state.current_angle = stabilized_angle
                
                # Feed angle data to data collector if recording
                if self.state.data_collector.is_recording and stabilized_angle is not None:
                    self.state.data_collector.update_angle(stabilized_angle)
                
                # Display camera windows if enabled
                if self.state.show_camera_windows:
                    # Draw red dot at center of raw image
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    
                    # Create display for threshold image
                    thresh_display = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                    
                    # Overlay detected line on the binary image
                    if line_coords is not None:
                        start_x, start_y, end_x, end_y = line_coords
                        cv2.line(thresh_display, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
                        cv2.circle(thresh_display, (start_x, start_y), 3, (255, 0, 0), -1)
                        
                        # Overlay angle text on raw image
                        if stabilized_angle is not None:
                            cv2.putText(frame, f"Angle: {stabilized_angle:.1f}°", (10, 30), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    else:
                        cv2.putText(frame, "No pointer detected", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    # Show windows
                    cv2.imshow('Live Feed', frame)
                    cv2.imshow('Preprocessed Image', thresh_display)
                    
                    # Handle window close events
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.state.show_camera_windows = False
                        cv2.destroyAllWindows()
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"❌ Camera processing error: {e}")
                time.sleep(0.1)
    
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
        
        # Clean up camera
        if self.state.camera and self.state.camera.isOpened():
            self.state.camera.release()
            print("✓ Camera released")
        
        # Stop any ongoing recording
        if self.state.data_collector.is_recording:
            self.state.data_collector.stop_recording()
            print("✓ Data recording stopped")
        
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
        
        # Camera and angle detection status
        camera_status = "Connected" if self.state.camera_enabled else "Disconnected"
        current_angle_display = f"{self.state.current_angle:.1f}°" if self.state.current_angle is not None else "N/A"
        camera_windows_status = "Visible" if self.state.show_camera_windows else "Hidden"
        
        # Data collection status
        recording_status = self.state.data_collector.get_status()
        recording_indicator = "🔴 Recording" if recording_status['is_recording'] else "⚪ Idle"
        
        print("POINTER ANGLE DETECTION:")
        print(f"  Camera Status: {camera_status}")
        print(f"  Current Pointer Angle: {current_angle_display}")
        print(f"  Camera Windows: {camera_windows_status}")
        print(f"  Recording: {recording_indicator} | Data Points: {recording_status['data_points']}")
        if self.state.camera_enabled:
            print(f"  Stabilization: Enabled (Factor: {ANGLE_SMOOTHING_FACTOR})")
        if recording_status['is_recording']:
            print(f"  Recording Time: {recording_status['elapsed_time']:.1f}s / 5.0s")
        
        # Display latest saved data and counter
        latest_data = self.state.data_collector.latest_saved_data
        if latest_data['diameter_mm'] is not None:
            status_icon = "✅ SAVED" if latest_data['saved'] else "❌ FAILED"
            direction = latest_data.get('direction', 'Unknown')
            consecutive_count = latest_data.get('consecutive_count', 0)
            count_display = f" | Count: {consecutive_count}" if consecutive_count > 0 else ""
            print(f"  Last Operation: {direction.upper()} | {latest_data['diameter_mm']}mm | {latest_data['target_steps']} steps | {latest_data['angle']:.2f}° | {status_icon}{count_display}")
        else:
            print(f"  Last Operation: No data recorded yet")
        
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
        print("  Recording: [P] Stop Recording (auto-stops after 5s)")
        print("  General:   [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit")
        print("=" * 60)
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            # --- Stepper Motor Controls (Step-based only) ---
            if key.char == 'a':  # Rotate CCW
                print(f"🚀 Starting CCW step move: Target steps = {self.state.target_steps}")
                
                # Capture initial angle for data collection
                initial_angle = self.state.current_angle
                
                # Step-based control for thin wires
                self.state.hardware.stepper.send_step_move_command(
                    self.state.target_steps, 'ccw'
                )
                
                # Check gripper closure before allowing data recording
                hw_status = self.state.hardware.get_status()
                gripper_closure = hw_status['gripper']['closure_percent']
                
                # Start camera data collection for 5 seconds (only if gripper is 100% closed)
                if gripper_closure < 100.0:
                    print("⚠ Data recording disabled: Gripper must be 100% closed for recording")
                elif self.state.camera_enabled and initial_angle is not None:
                    self.state.data_collector.start_recording(
                        'ccw', 
                        self.state.object_diameter_mm,
                        self.state.target_steps,
                        self.state.hardware.stepper.speed,
                        initial_angle
                    )
                elif not self.state.camera_enabled:
                    print("⚠ Camera not available for data collection")
                else:
                    print("⚠ No initial angle available for data collection")
                    
            elif key.char == 'd':  # Rotate CW
                print(f"🚀 Starting CW step move: Target steps = {self.state.target_steps}")
                
                # Capture initial angle for data collection
                initial_angle = self.state.current_angle
                
                # Step-based control for thin wires
                self.state.hardware.stepper.send_step_move_command(
                    self.state.target_steps, 'cw'
                )
                
                # Check gripper closure before allowing data recording
                hw_status = self.state.hardware.get_status()
                gripper_closure = hw_status['gripper']['closure_percent']
                
                # Start camera data collection for 5 seconds (only if gripper is 100% closed)
                if gripper_closure < 100.0:
                    print("⚠ Data recording disabled: Gripper must be 100% closed for recording")
                elif self.state.camera_enabled and initial_angle is not None:
                    self.state.data_collector.start_recording(
                        'cw', 
                        self.state.object_diameter_mm,
                        self.state.target_steps,
                        self.state.hardware.stepper.speed,
                        initial_angle
                    )
                elif not self.state.camera_enabled:
                    print("⚠ Camera not available for data collection")
                else:
                    print("⚠ No initial angle available for data collection")
            
            # --- Speed Control ---
            elif key.char == 'w':
                self.state.hardware.stepper.speed += 10
            elif key.char == 's':
                self.state.hardware.stepper.speed = max(10, self.state.hardware.stepper.speed - 10)
                
            # --- Target Step Adjustment ---
            elif key.char == 'e':
                self.state.target_steps += self.state.step_increment
                # No maximum limit for thin wire steps
                # Reset counter when target steps change
                self.state.data_collector.reset_consecutive_saves_counter()
            elif key.char == 'q':
                self.state.target_steps = max(self.state.min_steps, 
                                           self.state.target_steps - self.state.step_increment)
                # Reset counter when target steps change
                self.state.data_collector.reset_consecutive_saves_counter()
            
            # --- Wire Diameter Adjustment ---
            elif key.char == '1':
                self.state.update_diameter(0.2)
                # Reset counter when diameter changes
                self.state.data_collector.reset_consecutive_saves_counter()
            elif key.char == '2':
                self.state.update_diameter(0.5)
                # Reset counter when diameter changes
                self.state.data_collector.reset_consecutive_saves_counter()
            elif key.char == '3':
                self.state.update_diameter(1.0)
                # Reset counter when diameter changes
                self.state.data_collector.reset_consecutive_saves_counter()

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

            # --- Data Recording Controls ---
            elif key.char == 'p':  # Stop recording manually
                if self.state.data_collector.is_recording:
                    self.state.data_collector.stop_recording()
                    print("🛑 Recording stopped manually")
                else:
                    print("⚠ No recording in progress")

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
