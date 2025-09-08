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
import json
import struct
import pysoem
import ctypes
import collections

# Try to import numpy
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("Warning: numpy not available. Some functions will be disabled.")

# Try to import plotting libraries
try:
    import matplotlib.pyplot as plt
    from scipy.interpolate import interp1d
    from scipy.ndimage import gaussian_filter1d
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: matplotlib/scipy not available. Plotting functions will be disabled.")

from pynput import keyboard

from config import CONFIG, DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG, MANUAL_TILT_CONFIG
from hardware_manager import HardwareManager
from sensor_manager import SensorManager
from utils import get_steps_per_degree


def calculate_arrow_mesh(arrows, scale=1.0, grid_n=15):
    """
    Given a 2D vector field (H, W, 2), returns:
        - start_coords: (N, 2) array of (x, y) start points
        - magnitudes: (N,) array of arrow magnitudes
        - directions: (N, 2) array of (dx, dy) directions (unit vectors)
    The grid is sampled at grid_n x grid_n points.
    """
    if not NUMPY_AVAILABLE:
        print("Warning: numpy not available for mesh calculation")
        return [], [], []
    
    H, W = arrows.shape[:2]
    # grid_n is the number of arrows per axis
    y_idx = np.linspace(0, H-1, grid_n, dtype=int)
    x_idx = np.linspace(0, W-1, grid_n, dtype=int)
    grid_y, grid_x = np.meshgrid(y_idx, x_idx, indexing='ij')
    # Get start coordinates
    start_coords = np.stack([grid_x, grid_y], axis=-1).reshape(-1, 2)
    # Get arrow vectors at those points
    vectors = arrows[grid_y, grid_x] * scale  # shape (grid_n, grid_n, 2)
    vectors = vectors.reshape(-1, 2)
    # Magnitude
    magnitudes = np.linalg.norm(vectors, axis=1)
    # Directions (unit vectors, avoid division by zero)
    with np.errstate(divide='ignore', invalid='ignore'):
        directions = np.where(magnitudes[:, None] > 0, vectors / magnitudes[:, None], 0)
    return start_coords, magnitudes, directions


def save_force_data_to_json(timestamp, mean_fz, peak_intensity, deformation_mesh, shear_mesh):
    """
    Save force data to a JSON file with timestamp-based filename.
    
    Args:
        timestamp: Recording start timestamp
        mean_fz: Mean Fz value to 4 decimal places
        peak_intensity: Peak intensity to 3 decimal places
        deformation_mesh: List of dicts with coordinates, magnitude, and direction
        shear_mesh: List of dicts with coordinates, magnitude, and direction
    """
    # Create data structure
    force_data = {
        "recording_timestamp": timestamp,
        "mean_fz": round(mean_fz, 4),
        "peak_intensity": round(peak_intensity, 3),
        "deformation_mesh": deformation_mesh,
        "shear_mesh": shear_mesh
    }
    
    # Create data directory path
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data", "pull_force_100.csv")
    
    # Create data directory if it doesn't exist
    try:
        os.makedirs(data_dir, exist_ok=True)
    except Exception as e:
        print(f"❌ Error creating data directory: {e}")
        return
    
    # Create filename with timestamp
    timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(timestamp))
    filename = f"grip_pull_{timestamp_str}.json"
    
    # Create full file path
    file_path = os.path.join(data_dir, filename)
    
    # Save to JSON file
    try:
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(force_data, f, indent=2, ensure_ascii=False)
        print(f"✅ Force data saved to: {file_path}")
    except Exception as e:
        print(f"❌ Error saving force data: {e}")


class BotaSensorManager:
    """Manager for Bota force sensor integration"""
    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    SINC_LENGTH = 256
    
    def __init__(self, ifname="\\Device\\NPF_{6B61C18B-8290-4FF1-A5C0-3C01D5676AE1}"):
        self._ifname = ifname
        self._master = pysoem.Master()
        SlaveSet = collections.namedtuple('SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_mapping = {0: SlaveSet('BFT-MEDS-ECAT-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}
        
        # State variables
        self.connected = False
        self.running = False
        self.time_step = 1.0
        self.sampling_rate = None
        
        # Force data
        self.fz_offset = 0.0
        self.current_fz = 0.0
        self.last_mean_fz = None
        self.last_mean_intensity = None
        
        # Recording state
        self.recording = False
        self.fz_samples = []
        self.intensity_samples = []
        self.record_start_time = None
        self.plot_data_ready = None  # Will store data for main thread plotting
        self.last_snapshot_time = None  # Track last 1s snapshot time
        
        # Latest sensor data
        self.latest_data = {
            'status': 0,
            'warningsErrorsFatals': 0,
            'Fx': 0.0, 'Fy': 0.0, 'Fz': 0.0,
            'Mx': 0.0, 'My': 0.0, 'Mz': 0.0,
            'forceTorqueSaturated': 0,
            'temperature': 0.0
        }
        
        # Last message for status display
        self.last_message = "Not initialized"

    def bota_sensor_setup(self, slave_pos):
        """Configure the Bota sensor"""
        slave = self._master.slaves[slave_pos]
        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(self.SINC_LENGTH)))
        try:
            slave.sdo_write(0x8011, 0, struct.pack('h', 100))
        except Exception:
            pass
        sampling_rate = struct.unpack('h', slave.sdo_read(0x8011, 0))[0]
        if sampling_rate > 0:
            self.time_step = 1.0/float(sampling_rate)
        self.sampling_rate = sampling_rate

    def initialize(self):
        """Initialize the Bota sensor"""
        try:
            self._master.open(self._ifname)
            if self._master.config_init() > 0:
                for i, slave in enumerate(self._master.slaves):
                    if slave.man != self.BOTA_VENDOR_ID or slave.id != self._expected_slave_mapping[i].product_code:
                        self.last_message = f"Unexpected slave configuration"
                        return False
                    slave.config_func = self._expected_slave_mapping[i].config_func
                
                self._master.config_map()
                if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
                    self.last_message = "Failed to reach SAFEOP state"
                    return False
                
                self._master.state = pysoem.OP_STATE
                self._master.write_state()
                self._master.state_check(pysoem.OP_STATE, 50000)
                if self._master.state != pysoem.OP_STATE:
                    self.last_message = "Failed to reach OP state"
                    return False
                
                self.connected = True
                self.running = True
                self.last_message = "Connected and operational"
                return True
            else:
                self.last_message = "No slaves found"
                return False
        except Exception as e:
            self.last_message = f"Connection error: {str(e)}"
            return False

    def start_reader_thread(self, system_state):
        """Start the sensor reading thread"""
        if not self.connected:
            return None
        
        def reader_thread():
            try:
                while self.running and system_state.running:
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)
                    
                    # Parse sensor data
                    sensor_input_as_bytes = self._master.slaves[0].input
                    status = struct.unpack_from('B', sensor_input_as_bytes, 0)[0]
                    warningsErrorsFatals = struct.unpack_from('I', sensor_input_as_bytes, 1)[0]
                    Fx = struct.unpack_from('f', sensor_input_as_bytes, 5)[0]
                    Fy = struct.unpack_from('f', sensor_input_as_bytes, 9)[0]
                    Fz = struct.unpack_from('f', sensor_input_as_bytes, 13)[0]
                    Mx = struct.unpack_from('f', sensor_input_as_bytes, 17)[0]
                    My = struct.unpack_from('f', sensor_input_as_bytes, 21)[0]
                    Mz = struct.unpack_from('f', sensor_input_as_bytes, 25)[0]
                    forceTorqueSaturated = struct.unpack_from('H', sensor_input_as_bytes, 29)[0]
                    temperature = struct.unpack_from('f', sensor_input_as_bytes, 57)[0]
                    
                    # Apply Fz offset and store current value
                    self.current_fz = Fz - self.fz_offset
                    
                    # Update latest data
                    self.latest_data = {
                        'status': status,
                        'warningsErrorsFatals': warningsErrorsFatals,
                        'Fx': Fx, 'Fy': Fy, 'Fz': self.current_fz,
                        'Mx': Mx, 'My': My, 'Mz': Mz,
                        'forceTorqueSaturated': forceTorqueSaturated,
                        'temperature': temperature
                    }
                    
                    # Handle recording if active
                    # ORIGINAL 1-SECOND RECORDING LOGIC - COMMENTED OUT
                    # if self.recording:
                    #     self.fz_samples.append(self.current_fz)
                    #     
                    #     # Get visuotactile intensity if available
                    #     if system_state.sensors.visuotactile.connected:
                    #         intensity = system_state.sensors.visuotactile.max_depth_intensity
                    #         self.intensity_samples.append(intensity)
                    #     
                    #     # Check if recording duration reached
                    #     if (time.time() - self.record_start_time) >= 1.0:
                    #         self._finish_recording()
                    
                    # NEW 60-SECOND FZ RECORDING LOGIC WITH 1-SECOND JSON SNAPSHOTS
                    if self.recording:
                        current_time = time.time()
                        elapsed_time = current_time - self.record_start_time
                        self.fz_samples.append((elapsed_time, abs(self.current_fz)))
                        
                        # Check if 1 second has elapsed since last snapshot (only for first 30 seconds)
                        if int(elapsed_time) > self.last_snapshot_time and elapsed_time <= 30.0:
                            # Try to capture snapshot - only increment timer if successful
                            if self._capture_1s_snapshot(system_state, elapsed_time):
                                self.last_snapshot_time = int(elapsed_time)
                            # If snapshot failed (empty mesh), last_snapshot_time stays the same for retry
                        
                        # Check if 60-second recording duration reached
                        if elapsed_time >= 60.0:
                            self._finish_60s_recording()
                    
                    time.sleep(self.time_step)
                    
            except Exception as e:
                self.last_message = f"Reader error: {str(e)}"
            finally:
                self.connected = False
                try:
                    self._master.state = pysoem.INIT_STATE
                    self._master.write_state()
                    self._master.close()
                except:
                    pass
        
        thread = threading.Thread(target=reader_thread, name="BotaSensorThread", daemon=True)
        thread.start()
        return thread

    def zero_fz(self):
        """Zero the Fz reading (calibration)"""
        if self.connected:
            self.fz_offset = self.latest_data['Fz'] + self.fz_offset  # Account for current offset
            self.last_message = f"Fz zeroed at {self.fz_offset:.4f}N"
            return True
        return False

    def start_recording(self):
        """Start 60-second Fz recording with 1-second JSON snapshots (first 30s only)"""
        if self.connected and not self.recording:
            self.recording = True
            self.fz_samples = []  # Will store (time, abs_fz) tuples for CSV
            self.intensity_samples = []  # Not used in new logic but keeping for compatibility
            self.record_start_time = time.time()
            self.last_snapshot_time = 0  # Initialize snapshot timer
            self.last_message = "Recording started (60s with 30 snapshots in first 30s)"
            print("🔴 Started 60-second Fz recording with 1-second JSON snapshots (first 30s only)...")
            return True
        return False
    
    def stop_recording(self):
        """Stop and discard current recording session"""
        if self.recording:
            self.recording = False
            sample_count = len(self.fz_samples)
            snapshot_count = self.last_snapshot_time if self.last_snapshot_time else 0
            self.fz_samples = []  # Clear collected data
            self.intensity_samples = []  # Clear compatibility data
            self.record_start_time = None
            self.last_snapshot_time = None  # Reset snapshot timer
            self.last_message = "Recording stopped and discarded"
            print(f"⏹️ Recording stopped and discarded ({sample_count} samples, {snapshot_count} snapshots)")
            return True
        return False

    def _capture_1s_snapshot(self, system_state, elapsed_time):
        """Capture instantaneous snapshot at 1-second intervals and save to JSON
        
        Returns:
            bool: True if snapshot was successfully captured with valid mesh data, False otherwise
        """
        try:
            # Get instantaneous Fz value
            instantaneous_fz = abs(self.current_fz)
            
            # Get current visuotactile data if available
            if system_state.sensors.visuotactile.connected:
                try:
                    # Get current intensity
                    peak_intensity = system_state.sensors.visuotactile.max_depth_intensity
                    
                    # Get current deformation and shear data
                    deformation = system_state.sensors.visuotactile.get_latest_deformation()
                    shear = system_state.sensors.visuotactile.get_latest_shear()
                    
                    if deformation is not None and shear is not None:
                        # Calculate mesh data
                        if NUMPY_AVAILABLE:
                            # Calculate deformation mesh
                            def_coords, def_mags, def_dirs = calculate_arrow_mesh(
                                deformation, scale=20.0
                            )
                            
                            # Calculate shear mesh
                            shear_coords, shear_mags, shear_dirs = calculate_arrow_mesh(
                                shear, scale=20.0
                            )
                            
                            # Check if both meshes have valid data (non-empty)
                            if len(def_coords) > 0 and len(shear_coords) > 0:
                                # Prepare mesh data for JSON saving
                                deformation_mesh = []
                                for i in range(len(def_coords)):
                                    deformation_mesh.append({
                                        "coordinates": [int(def_coords[i][0]), int(def_coords[i][1])],
                                        "magnitude": float(def_mags[i]),
                                        "direction": [float(def_dirs[i][0]), float(def_dirs[i][1])]
                                    })
                                
                                shear_mesh = []
                                for i in range(len(shear_coords)):
                                    shear_mesh.append({
                                        "coordinates": [int(shear_coords[i][0]), int(shear_coords[i][1])],
                                        "magnitude": float(shear_mags[i]),
                                        "direction": [float(shear_dirs[i][0]), float(shear_dirs[i][1])]
                                    })
                                
                                # Save 1-second snapshot to JSON (only if both meshes are valid)
                                self._save_1s_snapshot_json(elapsed_time, instantaneous_fz, peak_intensity, 
                                                          deformation_mesh, shear_mesh)
                                return True  # Successfully captured snapshot with valid mesh data
                            else:
                                print(f"⚠ Warning: Empty mesh data at {elapsed_time:.0f}s - retrying next frame")
                                return False  # Failed due to empty mesh data
                        else:
                            print(f"⚠ Warning: numpy not available for mesh calculation at {elapsed_time:.0f}s")
                            return False  # Failed due to numpy unavailability
                    else:
                        print(f"⚠ Warning: visuotactile data not available at {elapsed_time:.0f}s - retrying next frame")
                        return False  # Failed due to missing visuotactile data
                except Exception as e:
                    print(f"⚠ Warning: Error capturing visuotactile data at {elapsed_time:.0f}s: {e} - retrying next frame")
                    return False  # Failed due to visuotactile error
            else:
                print(f"⚠ Warning: visuotactile sensor not connected at {elapsed_time:.0f}s - retrying next frame")
                return False  # Failed due to sensor disconnection
                
        except Exception as e:
            print(f"❌ Error capturing 1s snapshot at {elapsed_time:.0f}s: {e}")
            return False  # Failed due to general error

    def _save_1s_snapshot_json(self, elapsed_time, fz_value, peak_intensity, deformation_mesh, shear_mesh):
        """Save 1-second snapshot data to JSON file"""
        try:
            # Create data structure
            snapshot_data = {
                "recording_timestamp": self.record_start_time + elapsed_time,
                "elapsed_time_s": round(elapsed_time, 1),
                "fz": round(fz_value, 6),
                "peak_intensity": round(peak_intensity, 3),
                "deformation_mesh": deformation_mesh,
                "shear_mesh": shear_mesh
            }
            
            # Create data directory path
            data_dir = os.path.join(os.path.dirname(__file__), "..", "data", "pull_force_json")
            
            # Create data directory if it doesn't exist
            try:
                os.makedirs(data_dir, exist_ok=True)
            except Exception as e:
                print(f"❌ Error creating data directory: {e}")
                return
            
            # Create filename with timestamp and elapsed time
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.record_start_time))
            filename = f"snapshot_{timestamp_str}_{int(elapsed_time):02d}s.json"
            
            # Create full file path
            file_path = os.path.join(data_dir, filename)
            
            # Save to JSON file
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(snapshot_data, f, indent=2, ensure_ascii=False)
            
            print(f"📸 Snapshot saved: {int(elapsed_time)}s - Fz={fz_value:.4f}N")
            
        except Exception as e:
            print(f"❌ Error saving 1s snapshot JSON: {e}")

    def _finish_recording(self):
        """Finish recording and calculate means - ORIGINAL METHOD (COMMENTED OUT)"""
        # if self.recording:
        #     # Calculate means
        #     self.last_mean_fz = sum(self.fz_samples) / len(self.fz_samples) if self.fz_samples else 0.0
        #     self.last_mean_intensity = sum(self.intensity_samples) / len(self.intensity_samples) if self.intensity_samples else 0.0
        #     
        #     self.recording = False
        #     self.last_message = f"Recording complete: Fz={self.last_mean_fz:.4f}N, Intensity={self.last_mean_intensity:.3f}"
        pass

    def _finish_60s_recording(self):
        """Finish 60-second recording and prepare data for CSV export"""
        if self.recording and self.fz_samples:
            try:
                # Store data for main thread CSV processing
                self.plot_data_ready = {
                    'fz_samples': self.fz_samples.copy(),
                    'record_start_time': self.record_start_time,
                    'timestamp': time.time()
                }
                
                # Update status
                self.recording = False
                sample_count = len(self.fz_samples)
                final_fz = abs(self.fz_samples[-1][1]) if self.fz_samples else 0.0
                snapshot_count = self.last_snapshot_time
                self.last_message = f"60s recording complete: {sample_count} samples, {snapshot_count}/30 snapshots, final |Fz|={final_fz:.4f}N (CSV ready)"
                
                print(f"✅ 60-second recording complete!")
                print(f"   - {sample_count} data points collected for CSV")
                print(f"   - {snapshot_count}/30 JSON snapshots saved (first 30s only)")
                print(f"   - Final |Fz| value: {final_fz:.4f}N")
                print(f"   - Data prepared for CSV export")
                
            except Exception as e:
                self.recording = False
                self.last_message = f"Error processing recording: {e}"
                print(f"❌ Error processing 60s recording: {e}")
        else:
            self.recording = False
            self.last_message = "Recording finished but no data collected"
    
    def process_plot_data(self):
        """Process data and save to CSV file instead of plotting"""
        if self.plot_data_ready is None:
            return False
        
        print("🔄 Processing recording data...")
        
        try:
            # Extract stored data
            fz_samples = self.plot_data_ready['fz_samples']
            record_start_time = self.plot_data_ready['record_start_time']
            
            print(f"✓ Extracted {len(fz_samples)} samples from recording data")
            
            # Create CSV filename with timestamp
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(record_start_time))
            csv_filename = f"pull_force_{timestamp_str}.csv"
            csv_path = os.path.join(os.path.dirname(__file__), csv_filename)
            
            print(f"🔄 Saving data to: {csv_path}")
            
            try:
                # Save data to CSV file
                with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                    writer = csv.writer(csvfile)
                    
                    # Write header
                    writer.writerow(['timestamp', 'elapsed_time_s', 'abs_fz_N'])
                    
                    # Write data rows
                    for elapsed_time, abs_fz in fz_samples:
                        # Calculate absolute timestamp
                        absolute_timestamp = record_start_time + elapsed_time
                        
                        # Format timestamp with milliseconds (since strftime doesn't support %f with localtime)
                        dt_part = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(absolute_timestamp))
                        milliseconds = int((absolute_timestamp % 1) * 1000)
                        timestamp_formatted = f"{dt_part}.{milliseconds:03d}"
                        
                        writer.writerow([timestamp_formatted, f"{elapsed_time:.3f}", f"{abs_fz:.6f}"])
                
                print(f"✅ Data saved successfully to: {csv_path}")
                
                # Calculate final values for status
                final_fz = abs(fz_samples[-1][1]) if fz_samples else 0.0
                sample_count = len(fz_samples)
                
                # Update final status
                self.last_message = f"CSV saved: {sample_count} samples, final |Fz|={final_fz:.4f}N"
                
                print(f"   - {sample_count} data points saved")
                print(f"   - Final |Fz| value: {final_fz:.4f}N")
                print(f"   - CSV file: {csv_path}")
                
            except Exception as save_error:
                print(f"❌ Error saving CSV: {save_error}")
                self.last_message = f"Error saving CSV: {save_error}"
                return False
            
            # Clear the data
            self.plot_data_ready = None
            return True
            
        except Exception as e:
            print(f"❌ Error processing data: {e}")
            self.plot_data_ready = None
            return False

    def get_status(self):
        """Get current sensor status"""
        return {
            'connected': self.connected,
            'recording': self.recording,
            'fz_offset': self.fz_offset,
            'current_fz': self.current_fz,
            'last_mean_fz': self.last_mean_fz,
            'last_mean_intensity': self.last_mean_intensity,
            'latest_data': self.latest_data.copy(),
            'last_message': self.last_message,
            'sampling_rate': self.sampling_rate
        }

    def cleanup(self):
        """Clean up the sensor connection"""
        self.running = False
        if self.connected:
            try:
                self._master.state = pysoem.INIT_STATE
                self._master.write_state()
                self._master.close()
                self.connected = False
                self.last_message = "Disconnected"
            except Exception as e:
                self.last_message = f"Cleanup error: {str(e)}"


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
        self.bota_sensor = BotaSensorManager()
        
        # Thread references
        self.threads = []
        
        # Data recording state
        self.recorded_deformation_frames = []
        self.recorded_shear_frames = []
        self.was_recording = False
    
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
        
        # Initialize Bota sensor
        bota_status = self.state.bota_sensor.initialize()
        print(f"Bota sensor status: {self.state.bota_sensor.last_message}")
        
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
        # Start stepper reader thread - DISABLED FOR GRIP-ONLY EXPERIMENT
        # if self.state.hardware.stepper.connected:
        #     stepper_thread = self.state.hardware.stepper.start_reader_thread(self.state)
        #     if stepper_thread:
        #         self.state.threads.append(stepper_thread)
        #         print("✓ Stepper reader thread started")
        
        # Start sensor reader thread
        if self.state.sensors.visuotactile.connected:
            sensor_thread = self.state.sensors.visuotactile.start_reader_thread(self.state)
            if sensor_thread:
                self.state.threads.append(sensor_thread)
                print("✓ Sensor reader thread started")
        
        # Start Bota sensor thread
        if self.state.bota_sensor.connected:
            bota_thread = self.state.bota_sensor.start_reader_thread(self.state)
            if bota_thread:
                self.state.threads.append(bota_thread)
                print("✓ Bota sensor reader thread started")
        
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
        self.state.bota_sensor.cleanup()
        
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
        print("         GRIP-ONLY CONTROL SYSTEM")
        print("=" * 60)
        
        # Get hardware status
        hw_status = self.state.hardware.get_status()
        sensor_status = self.state.sensors.get_status()
        bota_status = self.state.bota_sensor.get_status()
        
        # Stepper motor status - DISABLED FOR GRIP-ONLY EXPERIMENT
        # cw_steps_per_deg = get_steps_per_degree(self.state.object_diameter_mm, 'cw')
        # ccw_steps_per_deg = get_steps_per_degree(self.state.object_diameter_mm, 'ccw')
        # 
        # print(f"ROTATION CONTROL ({self.state.control_mode.upper()}-Based):")
        # print(f"  Motor Position: M1={hw_status['stepper']['m1_pos']:<4} | M2={hw_status['stepper']['m2_pos']:<4}")
        # if self.state.control_mode == "angle":
        #     print(f"  Current Angle:  {hw_status['stepper']['current_angle']:>6.1f}°")
        #     print(f"  Target Angle:   {self.state.target_angle_deg:>6.1f}°")
        #     print(f"  Steps/Degree:   CW={cw_steps_per_deg:.3f} | CCW={ccw_steps_per_deg:.3f}")
        # else:
        #     print(f"  Target Steps:   {self.state.target_steps:>6}")
        # print(f"  Object Diameter: {self.state.object_diameter_mm}mm")
        # print(f"  Speed (steps/s): {hw_status['stepper']['speed']:<4}")
        # print(f"  Status: {'Connected' if hw_status['stepper']['connected'] else 'Disconnected'}")
        # print(f"  Last Message: {hw_status['stepper']['last_message']}")
        # 
        # print("-" * 60)
        
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
        
        # Bota Force Sensor status
        bota_connected = "Connected" if bota_status['connected'] else "Disconnected"
        recording_status = "🔴 Recording" if bota_status['recording'] else "⚪ Idle"
        
        print("BOTA FORCE SENSOR:")
        print(f"  Status: {bota_connected} | Recording: {recording_status}")
        if bota_status['connected']:
            latest_data = bota_status['latest_data']
            print(f"  Forces: Fx={latest_data['Fx']:>7.3f}N | Fy={latest_data['Fy']:>7.3f}N | Fz={latest_data['Fz']:>7.3f}N")
            print(f"  Torques: Mx={latest_data['Mx']:>6.3f}Nm | My={latest_data['My']:>6.3f}Nm | Mz={latest_data['Mz']:>6.3f}Nm")
            print(f"  Temperature: {latest_data['temperature']:.1f}°C | Offset: {bota_status['fz_offset']:.4f}N")
            
            # Show recording progress for 60s recording
            if bota_status['recording'] and hasattr(self.state.bota_sensor, 'record_start_time'):
                elapsed = time.time() - self.state.bota_sensor.record_start_time
                progress = min(elapsed / 60.0 * 100, 100)
                samples_count = len(self.state.bota_sensor.fz_samples)
                snapshots_count = self.state.bota_sensor.last_snapshot_time if self.state.bota_sensor.last_snapshot_time else 0
                print(f"  Recording Progress: {progress:.1f}% ({elapsed:.1f}s/60s) | Samples: {samples_count} | Snapshots: {snapshots_count}")
            
            # Show last recording results (commented out old logic)
            # if bota_status['last_mean_fz'] is not None:
            #     print(f"  Last Recording: Fz={bota_status['last_mean_fz']:.4f}N | Intensity={bota_status['last_mean_intensity']:.3f}")
        
        print(f"  Last Message: {bota_status['last_message']}")
        
        print("-" * 60)
        
        # Sensor status
        sensor_available = "Connected" if sensor_status['visuotactile']['connected'] else "Disconnected"
        # tilt_mode = "Manual" if MANUAL_TILT_CONFIG["enabled"] else "Auto"
        # tilt_value = MANUAL_TILT_CONFIG["current_value"] if MANUAL_TILT_CONFIG["enabled"] else (
        #     sensor_status['visuotactile']['angle_offset'] if sensor_status['visuotactile']['angle_offset'] is not None else 0.0
        # )
        print("DEPTH SENSOR:")
        print(f"  Max Intensity: {sensor_status['visuotactile']['max_intensity']:.3f}")
        print(f"  Baseline: {sensor_status['visuotactile']['baseline']:.3f} | Net: {sensor_status['visuotactile']['net_intensity']:.3f}")
        if sensor_status['visuotactile']['connected']:
            print(f"  Sensor FPS: {sensor_status['visuotactile']['sensor_fps']:.1f}")
        # print(f"  Tilt Mode: {tilt_mode} | Angle: {tilt_value:.1f}°")  # DISABLED FOR GRIP-ONLY EXPERIMENT
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
        # print("  Rotation:  [A] CCW | [D] CW | [W/S] Speed | [Q/E] Target ±5°/±50")  # DISABLED FOR GRIP-ONLY EXPERIMENT
        # print("  Diameter:  [1] 1mm | [2/3/4/5] 2-5mm")  # DISABLED FOR GRIP-ONLY EXPERIMENT
        print("  Gripper:   [O] Open | [C] Close (Adaptive)")
        print("  Sensor:    [B] Calibrate Baseline")
        print("  Bota:      [Z] Zero Fz | [SPACE] Record 60s & Export CSV (30 snapshots first 30s)")
        # print("  Tilt:      [J] Decrease | [K] Increase | [T] Toggle Mode")  # DISABLED FOR GRIP-ONLY EXPERIMENT
        # print("  General:   [H] Home Motors* | [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit")  # DISABLED FOR GRIP-ONLY EXPERIMENT
        # print("             *Homing only works when gripper is fully open")  # DISABLED FOR GRIP-ONLY EXPERIMENT
        print("  General:   [ESC] Quit")
        print("=" * 60)
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            # --- Stepper Motor Controls (Angle or Step based) ---
            # DISABLED FOR GRIP-ONLY EXPERIMENT
            # if key.char == 'a':  # Rotate CCW
            #     if self.state.control_mode == "angle":
            #         # Capture initial angle BEFORE sending move command to avoid race condition
            #         initial_angle = self.state.hardware.stepper.get_current_angle_stable()
            #         print(f"🚀 Starting CCW rotation: Initial angle = {initial_angle:.1f}°, Target = {self.state.target_angle_deg:.1f}°")
            #         
            #         # Angle-based control for diameters 2-5mm
            #         self.state.hardware.stepper.send_angle_move_command(
            #             self.state.target_angle_deg, 'ccw', self.state.object_diameter_mm
            #         )
            #     else:
            #         # Capture initial angle BEFORE sending move command to avoid race condition
            #         initial_angle = self.state.hardware.stepper.get_current_angle_stable()
            #         print(f"🚀 Starting CCW step move: Initial angle = {initial_angle:.1f}°, Target steps = {self.state.target_steps}")
            #         
            #         # Step-based control for diameters ≤1mm
            #         self.state.hardware.stepper.send_step_move_command(
            #             self.state.target_steps, 'ccw'
            #         )
            #         
            # elif key.char == 'd':  # Rotate CW
            #     if self.state.control_mode == "angle":
            #         # Capture initial angle BEFORE sending move command to avoid race condition
            #         initial_angle = self.state.hardware.stepper.get_current_angle_stable()
            #         print(f"🚀 Starting CW rotation: Initial angle = {initial_angle:.1f}°, Target = {self.state.target_angle_deg:.1f}°")
            #         
            #         # Angle-based control for diameters 2-5mm
            #         self.state.hardware.stepper.send_angle_move_command(
            #             self.state.target_angle_deg, 'cw', self.state.object_diameter_mm
            #         )
            #     else:
            #         # Capture initial angle BEFORE sending move command to avoid race condition
            #         initial_angle = self.state.hardware.stepper.get_current_angle_stable()
            #         print(f"🚀 Starting CW step move: Initial angle = {initial_angle:.1f}°, Target steps = {self.state.target_steps}")
            #         
            #         # Step-based control for diameters ≤1mm
            #         self.state.hardware.stepper.send_step_move_command(
            #             self.state.target_steps, 'cw'
            #         )
            
            # --- Speed Control ---
            # DISABLED FOR GRIP-ONLY EXPERIMENT
            # elif key.char == 'w':
            #     self.state.hardware.stepper.speed += 10
            # elif key.char == 's':
            #     self.state.hardware.stepper.speed = max(10, self.state.hardware.stepper.speed - 10)
                
            # --- Target Adjustment (Angle or Steps) ---
            # DISABLED FOR GRIP-ONLY EXPERIMENT
            # elif key.char == 'e':
            #     if self.state.control_mode == "angle":
            #         self.state.target_angle_deg += CONFIG["angle_increment_deg"]
            #         self.state.target_angle_deg = min(self.state.target_angle_deg, CONFIG["max_angle_deg"])
            #     else:
            #         self.state.target_steps += CONFIG["step_increment"]
            #         self.state.target_steps = min(self.state.target_steps, CONFIG["max_steps_movement"])
            # elif key.char == 'q':
            #     if self.state.control_mode == "angle":
            #         self.state.target_angle_deg = max(CONFIG["angle_increment_deg"], 
            #                                        self.state.target_angle_deg - CONFIG["angle_increment_deg"])
            #     else:
            #         self.state.target_steps = max(CONFIG["step_increment"], 
            #                                    self.state.target_steps - CONFIG["step_increment"])
            
            # --- Object Diameter Adjustment ---
            # DISABLED FOR GRIP-ONLY EXPERIMENT
            # elif key.char == '1':
            #     self.state.update_diameter_and_control_mode(1.0)
            # elif key.char == '2':
            #     self.state.update_diameter_and_control_mode(2)
            # elif key.char == '3':
            #     self.state.update_diameter_and_control_mode(3)
            # elif key.char == '4':
            #     self.state.update_diameter_and_control_mode(4)
            # elif key.char == '5':
            #     self.state.update_diameter_and_control_mode(5)

            # --- Gripper Controls ---
            if key.char == 'o':  # Open gripper fully
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

            # --- Bota Sensor Controls ---
            elif key.char == 'z':  # Zero Fz force
                if self.state.bota_sensor.connected:
                    self.state.bota_sensor.zero_fz()
                else:
                    print("❌ Bota sensor not connected")

            # --- Tilt Controls ---
            # DISABLED FOR GRIP-ONLY EXPERIMENT
            # elif key.char == 'j':  # Decrease manual tilt
            #     self.state.sensors.decrement_manual_tilt()
            # elif key.char == 'k':  # Increase manual tilt
            #     self.state.sensors.increment_manual_tilt()
            # elif key.char == 't':  # Toggle tilt mode
            #     self.state.sensors.toggle_manual_tilt_mode()

            # --- Utility Commands ---
            # DISABLED FOR GRIP-ONLY EXPERIMENT
            # elif key.char == 'h':  # Home stepper motors (safety constraint: gripper must be fully open)
            #     gripper_closure = self.state.hardware.gripper.gripper_closure_percent
            #     if gripper_closure <= 5.0:  # Allow small tolerance for "fully open" (≤5%)
            #         if self.state.hardware.stepper.connected:
            #             print(f"🏠 Starting Python homing sequence...")
            #             print(f"⚠ Please wait ~6 seconds for homing to complete...")
            #             
            #             # Perform Python-based homing sequence
            #             homing_speed = CONFIG["homing_speed"]
            #             
            #             # Step 1: Move both motors to maximum position (1000)
            #             print(f"   Step 1/3: Moving to max position (1000, 1000)")
            #             self.state.hardware.stepper.send_move_command(1000, 1000, homing_speed)
            #             time.sleep(1.5)  # Increased delay to ensure movement completes
            #             
            #             # Step 2: Move both motors to minimum position (0)
            #             print(f"   Step 2/3: Moving to min position (0, 0)")
            #             self.state.hardware.stepper.send_move_command(0, 0, homing_speed)
            #             time.sleep(2.5)  # Increased delay to ensure movement completes
            #             
            #             # Step 3: Move both motors to center position (500, 500)
            #             print(f"   Step 3/3: Moving to center position (500, 500)")
            #             self.state.hardware.stepper.send_move_command(500, 500, homing_speed)
            #             time.sleep(1.0)  # Allow time for final positioning
            #             
            #             # Reset angle tracking to zero
            #             self.state.hardware.stepper.current_angle_deg = 0.0
            #             
            #             self.state.hardware.stepper.last_message = "Python homing sequence completed"
            #             print(f"✅ Homing sequence completed - motors at center, angle reset to 0°")
            #         else:
            #             print(f"❌ Cannot home: Stepper motors not connected")
            #     else:
            #         print(f"⚠ Motor homing BLOCKED: Gripper must be fully open (currently {gripper_closure:.1f}% closed)")
            #         print(f"📋 Press [O] to open gripper first, then retry homing with [H]")
            #         self.state.hardware.stepper.last_message = f"Homing blocked: gripper {gripper_closure:.1f}% closed"
            # elif key.char == 'x':  # Reset stepper position to center and angle
            #     self.state.hardware.stepper.reset_to_center()
            pass  # Placeholder for disabled commands

        except AttributeError:
            # Handle special keys
            # DISABLED FOR GRIP-ONLY EXPERIMENT
            # if key == keyboard.Key.space:
            #     # Stop all motors
            #     self.state.hardware.stepper.stop_motors()
            #     self.state.hardware.stepper.last_message = "STOP command sent"
            #     self.state.hardware.gripper.last_message = "Manual stop requested"
            if key == keyboard.Key.space:
                # Handle Bota sensor recording - start or interrupt
                if self.state.bota_sensor.connected:
                    if not self.state.bota_sensor.recording:
                        # Start new recording
                        self.state.bota_sensor.start_recording()
                        # Clear any old visuotactile frames (not used in new logic but keeping for safety)
                        self.state.recorded_deformation_frames = []
                        self.state.recorded_shear_frames = []
                        print("🔴 Started 60-second Fz recording (with 30 snapshots in first 30s)...")
                    else:
                        # Interrupt current recording
                        self.state.bota_sensor.stop_recording()
                        # Clear any recorded frames
                        self.state.recorded_deformation_frames = []
                        self.state.recorded_shear_frames = []
                        print("⏹️ Recording interrupted and data discarded. Ready for new recording.")
                else:
                    print("❌ Cannot control recording: Bota sensor not connected")
            elif key == keyboard.Key.esc:
                # Stop the listener and the program
                self.state.running = False
                return False
    
    def _process_recorded_data(self):
        """Process recorded visuotactile and force data and save to JSON"""
        print("\n--- Recording Finished ---")
        
        if not NUMPY_AVAILABLE:
            print("❌ Cannot process mesh data: numpy not available")
            return
        
        if (self.state.recorded_deformation_frames and 
            self.state.recorded_shear_frames and 
            self.state.bota_sensor.last_mean_fz is not None):
            
            try:
                # Use the last frame for mesh analysis
                last_deformation_frame = self.state.recorded_deformation_frames[-1]
                last_shear_frame = self.state.recorded_shear_frames[-1]

                # Calculate deformation mesh
                def_coords, def_mags, def_dirs = calculate_arrow_mesh(
                    last_deformation_frame, scale=20.0
                )
                
                # Calculate shear mesh
                shear_coords, shear_mags, shear_dirs = calculate_arrow_mesh(
                    last_shear_frame, scale=20.0
                )
                
                # Prepare mesh data for JSON saving
                deformation_mesh = []
                for i in range(len(def_coords)):
                    deformation_mesh.append({
                        "coordinates": [int(def_coords[i][0]), int(def_coords[i][1])],
                        "magnitude": float(def_mags[i]),
                        "direction": [float(def_dirs[i][0]), float(def_dirs[i][1])]
                    })
                
                shear_mesh = []
                for i in range(len(shear_coords)):
                    shear_mesh.append({
                        "coordinates": [int(shear_coords[i][0]), int(shear_coords[i][1])],
                        "magnitude": float(shear_mags[i]),
                        "direction": [float(shear_dirs[i][0]), float(shear_dirs[i][1])]
                    })
                
                # Get recording data
                recording_timestamp = self.state.bota_sensor.record_start_time
                mean_fz = self.state.bota_sensor.last_mean_fz
                peak_intensity = self.state.bota_sensor.last_mean_intensity
                
                # Save force data to JSON
                save_force_data_to_json(
                    recording_timestamp, mean_fz, peak_intensity, 
                    deformation_mesh, shear_mesh
                )
                
                print("✅ Data processing and saving completed")
                
            except Exception as e:
                print(f"❌ Error processing recorded data: {e}")
            finally:
                # Clear recorded frames
                self.state.recorded_deformation_frames.clear()
                self.state.recorded_shear_frames.clear()
        else:
            print("⚠ Recording finished, but insufficient data captured")
            # Clear any partial data
            self.state.recorded_deformation_frames.clear()
            self.state.recorded_shear_frames.clear()
    
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
                    
                    # OLD VISUOTACTILE FRAME RECORDING LOGIC - COMMENTED OUT
                    # Handle visuotactile frame recording and processing
                    # is_recording_now = self.state.bota_sensor.recording
                    # 
                    # # Capture frames during recording
                    # if is_recording_now and self.state.sensors.visuotactile.connected:
                    #     try:
                    #         # Get current deformation and shear data
                    #         deformation = self.state.sensors.visuotactile.get_latest_deformation()
                    #         shear = self.state.sensors.visuotactile.get_latest_shear()
                    #         
                    #         if deformation is not None and shear is not None:
                    #             self.state.recorded_deformation_frames.append(deformation)
                    #             self.state.recorded_shear_frames.append(shear)
                    #     except Exception as e:
                    #         print(f"Warning: Error capturing visuotactile frames: {e}")
                    # 
                    # # Process frames when recording finishes
                    # if self.state.was_recording and not is_recording_now:
                    #     self._process_recorded_data()
                    # 
                    # self.state.was_recording = is_recording_now
                    
                    # Check for adaptive gripping conditions
                    if (self.state.hardware.gripper.is_gripping and 
                        self.state.sensors.visuotactile.connected):
                        net_intensity = self.state.sensors.visuotactile.net_intensity
                        if self.state.sensors.check_gripping_condition(net_intensity):
                            self.state.hardware.gripper.handle_object_detection()
                    
                    # Process any pending CSV data (must be done in main thread)
                    if self.state.bota_sensor.plot_data_ready is not None:
                        print("🔄 CSV data ready detected in main loop, processing...")
                        self.state.bota_sensor.process_plot_data()
                    
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
