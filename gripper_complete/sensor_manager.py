"""
Sensor Manager for the Gripper Control System
Manages all sensor interactions (Visuotactile and Encoder)
"""

import time
import threading
import sys
import os

# Try to import numpy, but don't fail if it's not available
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("Warning: numpy not available. Some sensor functions will be disabled.")

# Try to import cv2, but don't fail if it's not available
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: cv2 not available. Image display functions will be disabled.")

# Try to import minimalmodbus, but don't fail if it's not available
try:
    import minimalmodbus
    MINIMALMODBUS_AVAILABLE = True
except ImportError:
    MINIMALMODBUS_AVAILABLE = False
    print("Warning: minimalmodbus not available. Encoder functions will be disabled.")

# Add paths for local imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'daimon', 'dmrobotics'))

from config import (
    CONFIG, SINGLE_TURN_RESOLUTION, RECORDING_CONFIG, 
    DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG
)
from utils import (
    get_steps_per_degree, save_encoder_data_to_csv, 
    create_encoder_plot, calculate_low_velocity_point
)

# Import sensor functionality
try:
    from dmrobotics import Sensor, put_arrows_on_image
    SENSOR_AVAILABLE = True
except ImportError:
    print("Warning: Sensor library not available. Depth sensing disabled.")
    SENSOR_AVAILABLE = False


class DaimonManager:
    """Manages Daimon visuotactile sensor operations"""
    
    def __init__(self):
        self.sensor = None
        self.connected = False
        self.last_message = ""
        self.max_depth_intensity = 0.0
        self.baseline_intensity = 0.0
        self.net_intensity = 0.0
        self.sensor_fps = 0.0
        self.frame_count = 0
        self.fps_start_time = 0.0
        
    def connect(self):
        """Initialize and configure the depth sensor"""
        if not SENSOR_AVAILABLE:
            self.last_message = "Sensor library not available"
            return False
        
        try:
            self.sensor = Sensor(CONFIG["sensor_serial_id"])
            self.connected = True
            self.last_message = "Sensor connected successfully"
            print("✓ Depth sensor connected")
            return True
        except Exception as e:
            self.last_message = f"ERR: Sensor setup failed: {e}"
            print("❌ Depth sensor setup failed")
            return False
    
    def disconnect(self):
        """Disconnect sensor and close windows"""
        if self.sensor and self.connected:
            try:
                self.close_sensor_windows()
                self.sensor.disconnect()
                print("✓ Sensor disconnected")
            except:
                pass
        self.connected = False
    
    def get_max_depth_intensity(self):
        """Get the sensor depth image and return the maximum intensity"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return 0.0
        
        if not NUMPY_AVAILABLE:
            self.last_message = "Numpy not available for depth processing"
            return 0.0
        
        try:
            depth = self.sensor.getDepth()
            
            if depth is not None and depth.size > 0:
                max_intensity = float(np.max(depth))
                self.max_depth_intensity = max_intensity
                self.net_intensity = max_intensity - self.baseline_intensity
                self.last_message = f"Max: {max_intensity:.3f}, Net: {self.net_intensity:.3f}"
                return max_intensity
            else:
                self.last_message = "No depth data available"
                return 0.0
                
        except Exception as e:
            self.last_message = f"ERR: Failed to get depth intensity: {e}"
            return 0.0
    
    def calibrate_baseline_intensity(self, samples=10):
        """Calibrate the baseline intensity when gripper is fully open"""
        if not self.sensor or not SENSOR_AVAILABLE:
            self.last_message = "ERR: Sensor not available for calibration"
            return False
        
        try:
            total_intensity = 0.0
            valid_samples = 0
            
            for i in range(samples):
                intensity = self.get_max_depth_intensity()
                if intensity > 0.0:
                    total_intensity += intensity
                    valid_samples += 1
                time.sleep(0.1)
            
            if valid_samples > 0:
                self.baseline_intensity = total_intensity / valid_samples
                self.net_intensity = 0.0
                self.last_message = f"Baseline calibrated: {self.baseline_intensity:.3f}"
                print(f"✓ Baseline intensity calibrated: {self.baseline_intensity:.3f}")
                return True
            else:
                self.last_message = "ERR: No valid samples for calibration"
                return False
                
        except Exception as e:
            self.last_message = f"ERR: Calibration failed: {e}"
            return False
    
    def display_sensor_images(self):
        """Display concatenated sensor images"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return
        
        if not CV2_AVAILABLE or not NUMPY_AVAILABLE:
            return
        
        try:
            # Get sensor data
            img = self.sensor.getRawImage()
            depth = self.sensor.getDepth()
            deformation = self.sensor.getDeformation2D()
            shear = self.sensor.getShear()
            
            # Create depth image with color mapping
            depth_img = cv2.applyColorMap((depth * 0.25 * 255.0).astype('uint8'), cv2.COLORMAP_HOT)
            
            # Create black background for deformation and shear arrows
            black_img = np.zeros_like(img)
            black_img = np.stack([black_img] * 3, axis=-1)
            
            # Create deformation and shear images
            deformation_img = put_arrows_on_image(black_img.copy(), deformation * 20)
            shear_img = put_arrows_on_image(black_img.copy(), shear * 20)
            
            # Ensure all images have the same height for concatenation
            height = depth_img.shape[0]
            width = depth_img.shape[1]
            
            # Resize if needed
            if deformation_img.shape[:2] != (height, width):
                deformation_img = cv2.resize(deformation_img, (width, height))
            if shear_img.shape[:2] != (height, width):
                shear_img = cv2.resize(shear_img, (width, height))
            
            # Add text labels
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            font_color = (255, 255, 255)
            thickness = 2
            
            cv2.putText(depth_img, 'Depth', (10, 30), font, font_scale, font_color, thickness)
            cv2.putText(deformation_img, 'Deformation', (10, 30), font, font_scale, font_color, thickness)
            cv2.putText(shear_img, 'Shear', (10, 30), font, font_scale, font_color, thickness)
            
            # Concatenate images horizontally
            combined_img = np.hstack([depth_img, deformation_img, shear_img])
            
            # Display the combined image
            cv2.imshow('Sensor Data - Depth | Deformation | Shear', combined_img)
            
            # Update FPS calculation
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.fps_start_time > DISPLAY_CONFIG["sensor_fps_calculation_interval"]:
                self.sensor_fps = self.frame_count / (current_time - self.fps_start_time)
                self.frame_count = 0
                self.fps_start_time = current_time
            
            # Check for window close events
            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                self.sensor.reset()
                self.last_message = "Sensor reset via image window"
            
        except Exception as e:
            self.last_message = f"ERR: Image display failed: {e}"
    
    def close_sensor_windows(self):
        """Close the sensor image window"""
        if not CV2_AVAILABLE:
            return
        try:
            cv2.destroyWindow('Sensor Data - Depth | Deformation | Shear')
        except:
            pass
    
    def start_reader_thread(self, state):
        """Start background thread for reading sensor data"""
        if self.sensor and self.connected:
            self.fps_start_time = time.time()
            self.frame_count = 0
            
            reader = threading.Thread(target=self._reader_loop, args=(state,))
            reader.daemon = True
            reader.start()
            return reader
        return None
    
    def _reader_loop(self, state):
        """Background thread for reading sensor data and displaying images"""
        while state.running:
            try:
                if self.sensor and self.connected:
                    self.get_max_depth_intensity()
                    self.display_sensor_images()
                    
                    # Read more frequently when gripping is active
                    if state.gripper.is_gripping:
                        time.sleep(0.01)  # 100Hz when gripping
                    else:
                        time.sleep(0.05)   # 20Hz when not gripping
            except Exception as e:
                self.last_message = f"ERR: Sensor reading failed: {e}"
                self.connected = False
                break
        
        self.close_sensor_windows()


class RotaryEncoderManager:
    """Manages rotary encoder operations"""
    
    def __init__(self):
        self.instrument = None
        self.connected = False
        self.last_message = ""
        self.is_ccw = False
        self.is_recording = False
        self.recorded_data = []
        self.recording_start_time = None
        
        # Recording metadata
        self.recording_diameter = 0
        self.recording_target_angle = 0.0
        self.recording_grip_strength = 0.0
        self.recording_initial_angle = 0.0
        self.recording_direction = 'cw'
        
    def connect(self):
        """Initialize encoder connection"""
        if not MINIMALMODBUS_AVAILABLE:
            self.last_message = "Minimalmodbus not available"
            return False
            
        try:
            self.instrument = minimalmodbus.Instrument(
                CONFIG["encoder_port"], 
                CONFIG["encoder_slave_address"]
            )
            self.instrument.serial.baudrate = CONFIG["encoder_baudrate"]
            self.instrument.serial.bytesize = 8
            self.instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
            self.instrument.serial.stopbits = 1
            self.instrument.serial.timeout = 0.5
            self.instrument.mode = minimalmodbus.MODE_RTU
            
            # Test connection
            test_angle, _, _, _, _ = self.read_encoder_data()
            if test_angle is not None:
                self.connected = True
                self.last_message = f"Connected to {CONFIG['encoder_port']}"
                print(f"✓ Rotary encoder connected to {CONFIG['encoder_port']}")
                
                # Set encoder direction to CW (fixed)
                self.set_encoder_direction(False)
                return True
            else:
                print(f"❌ Could not read from encoder on {CONFIG['encoder_port']}")
                self.last_message = "Connection failed - no data"
                return False
                
        except Exception as e:
            print(f"❌ Could not connect to encoder on {CONFIG['encoder_port']}: {e}")
            self.last_message = f"Connection failed: {e}"
            return False
    
    def disconnect(self):
        """Disconnect encoder"""
        if self.connected:
            try:
                if self.is_recording:
                    self.stop_encoder_recording()
                print("✓ Encoder disconnected")
            except:
                pass
        self.connected = False
    
    def read_encoder_data(self):
        """Read and decode encoder data"""
        try:
            total_encoded_value = self.instrument.read_long(
                CONFIG["encoder_read_register"], 3, False
            )
            single_turn_value = total_encoded_value & 0x7FFF
            turn_count = total_encoded_value >> 15
            angle = (single_turn_value / SINGLE_TURN_RESOLUTION) * 360.0
            
            direction_str = "CCW" if self.is_ccw else "CW"
            current_time = time.time()
            
            # Record data if recording is active
            if self.is_recording and self.recording_start_time is not None:
                elapsed_time = current_time - self.recording_start_time
                self.recorded_data.append([
                    time.strftime('%H:%M:%S.%f')[:-3],
                    elapsed_time,
                    angle,
                    turn_count,
                    single_turn_value,
                    total_encoded_value,
                    direction_str
                ])
            
            return angle, turn_count, single_turn_value, total_encoded_value, direction_str
            
        except Exception as e:
            self.last_message = f"Encoder error: {e}"
            return None, None, None, None, None
    
    def zero_encoder(self):
        """Send the command to zero the encoder"""
        try:
            print("\nSending zeroing command...")
            self.instrument.write_register(
                registeraddress=CONFIG["encoder_zero_register"],
                value=1,
                functioncode=6
            )
            print("Zeroing command sent successfully!")
        except Exception as e:
            print(f"Failed to send zeroing command: {e}")
    
    def set_encoder_direction(self, set_to_ccw):
        """Send the command to set the encoder direction"""
        try:
            value_to_write = 1 if set_to_ccw else 0
            direction_str = "Counter-Clockwise (CCW)" if set_to_ccw else "Clockwise (CW)"
            
            print(f"\nSetting direction to {direction_str}...")
            self.instrument.write_register(
                registeraddress=CONFIG["encoder_direction_register"],
                value=value_to_write,
                functioncode=6
            )
            print("Direction set successfully!")
            self.is_ccw = set_to_ccw
            return set_to_ccw
        except Exception as e:
            print(f"Failed to set direction: {e}")
            return not set_to_ccw
    
    def start_encoder_recording(self, direction=None, gripper_closure_percent=0, initial_angle=None):
        """Start recording encoder data"""
        if gripper_closure_percent <= 0:
            print(f"⚠ Recording disabled: Gripper is open ({gripper_closure_percent:.1f}% closed)")
            return
        
        if not self.is_recording:
            self.is_recording = True
            self.recorded_data = []
            self.recording_start_time = time.time()
            
            self.recording_direction = direction if direction is not None else ('ccw' if self.is_ccw else 'cw')
            
            # Use provided initial angle (motor position-based) or fallback to encoder reading
            if initial_angle is not None:
                self.recording_initial_angle = initial_angle
                print(f"📐 Using motor position-based initial angle: {initial_angle:.1f}°")
            elif self.instrument and self.connected:
                encoder_angle, _, _, _, _ = self.read_encoder_data()
                self.recording_initial_angle = encoder_angle if encoder_angle is not None else 0.0
                print(f"📐 Using encoder reading as initial angle: {self.recording_initial_angle:.1f}°")
            else:
                self.recording_initial_angle = 0.0
                print(f"📐 Using default initial angle: {self.recording_initial_angle:.1f}°")
                
            print(f"\n🎙️  Encoder recording started at {time.strftime('%H:%M:%S')}")
    
    def stop_encoder_recording(self):
        """Stop recording encoder data and save to file"""
        if self.is_recording:
            self.is_recording = False
            recording_duration = time.time() - self.recording_start_time
            print(f"\n⏹️  Encoder recording stopped. Duration: {recording_duration:.2f}s")
            print(f"📊 Collected {len(self.recorded_data)} data points")
            
            if self.recorded_data:
                try:
                    self._process_and_save_recording_data(recording_duration)
                except Exception as e:
                    print(f"❌ Error during data processing: {e}")
            else:
                print("⚠ No data collected during recording")
            
            self.recorded_data = []
            self.recording_start_time = None
    
    def _process_and_save_recording_data(self, duration):
        """Process recorded data and save to CSV/plot"""
        if not self.recorded_data:
            return
        
        # Calculate low velocity point
        low_velocity_time, low_velocity_angle = calculate_low_velocity_point(self.recorded_data)
        
        # Use the low velocity angle as measured angle, or fallback to last reading
        measured_angle = low_velocity_angle if low_velocity_angle is not None else (
            self.recorded_data[-1][2] if self.recorded_data else 0.0
        )
        
        # For CCW rotation, calculate the positive angle change from start to finish
        if self.recording_direction == 'ccw':
            angle_change = measured_angle - self.recording_initial_angle
            
            # Handle wrap-around cases (0°/360° boundary)
            if angle_change > 180:
                angle_change -= 360
            elif angle_change < -180:
                angle_change += 360
            
            # For CCW, we want positive angle change, so negate if negative
            if angle_change < 0:
                angle_change = -angle_change
            
            measured_angle = angle_change
        
        # Calculate error
        error = measured_angle - self.recording_target_angle
        
        # Check if error is within acceptable range
        if abs(error) <= RECORDING_CONFIG["error_threshold_degrees"]:
            # Calculate steps using calibration data
            steps_per_degree = get_steps_per_degree(self.recording_diameter, self.recording_direction)
            calculated_steps = self.recording_target_angle * steps_per_degree
            calculated_steps = round(calculated_steps / 0.0625) * 0.0625
            
            # Prepare data row for CSV
            data_row = [
                self.recording_diameter,
                round(self.recording_grip_strength, 4),
                self.recording_direction,
                round(calculated_steps, 4),
                round(self.recording_initial_angle, 2),
                round(self.recording_target_angle, 2),
                round(measured_angle, 2),
                round(error, 2)
            ]
            
            # Save to CSV
            save_encoder_data_to_csv(data_row)
            
            # Create plot
            create_encoder_plot(
                self.recorded_data, duration, 
                self.recording_diameter, self.recording_target_angle, 
                self.recording_direction, low_velocity_time, low_velocity_angle
            )
        else:
            print(f"⚠ Data discarded: error {error:.1f}° exceeds ±{RECORDING_CONFIG['error_threshold_degrees']}° threshold")
    
    def start_reader_thread(self, state):
        """Start background thread for reading encoder data"""
        if self.instrument and self.connected:
            reader = threading.Thread(target=self._reader_loop, args=(state,))
            reader.daemon = True
            reader.start()
            return reader
        return None
    
    def _reader_loop(self, state):
        """Background thread for reading encoder data"""
        while state.running:
            try:
                if self.instrument and self.connected:
                    # Read encoder data
                    angle, turn_count, single_turn, raw_value, direction = self.read_encoder_data()
                    if angle is not None:
                        self.last_message = f"Angle: {angle:.2f}° | Turns: {turn_count} | Dir: {direction}"
                    
                    # Check if recording should stop
                    if self.is_recording and self.recording_start_time is not None:
                        elapsed_time = time.time() - self.recording_start_time
                        if elapsed_time >= RECORDING_CONFIG["duration_seconds"]:
                            self.stop_encoder_recording()
                    
                    time.sleep(1.0 / RECORDING_CONFIG["sampling_rate_hz"])
            except Exception as e:
                self.last_message = f"ERR: Encoder reading failed: {e}"
                self.connected = False
                break


class SensorManager:
    """Main sensor manager that coordinates all sensor systems"""
    
    def __init__(self):
        self.visuotactile = DaimonManager()
        self.encoder = RotaryEncoderManager()
        self.available = {
            'visuotactile': SENSOR_AVAILABLE,
            'encoder': False
        }
    
    def initialize(self):
        """Initialize all sensor connections"""
        print("Initializing sensor connections...")
        
        # Initialize visuotactile sensor
        if self.available['visuotactile']:
            self.available['visuotactile'] = self.visuotactile.connect()
            if self.available['visuotactile']:
                # Calibrate baseline intensity
                print("Calibrating baseline intensity...")
                if self.visuotactile.calibrate_baseline_intensity():
                    print("✓ Baseline intensity calibrated")
                else:
                    print("⚠ Baseline calibration failed")
        
        # Initialize encoder
        self.available['encoder'] = self.encoder.connect()
        
        return self.available
    
    def cleanup(self):
        """Clean up all sensor connections"""
        print("Cleaning up sensor connections...")
        self.visuotactile.disconnect()
        self.encoder.disconnect()
    
    def get_status(self):
        """Get status of all sensor components"""
        return {
            'visuotactile': {
                'connected': self.visuotactile.connected,
                'last_message': self.visuotactile.last_message,
                'max_intensity': self.visuotactile.max_depth_intensity,
                'baseline': self.visuotactile.baseline_intensity,
                'net_intensity': self.visuotactile.net_intensity,
                'sensor_fps': self.visuotactile.sensor_fps
            },
            'encoder': {
                'connected': self.encoder.connected,
                'last_message': self.encoder.last_message,
                'is_recording': self.encoder.is_recording,
                'data_count': len(self.encoder.recorded_data) if self.encoder.is_recording else 0
            }
        }
    
    def check_gripping_condition(self, net_intensity):
        """Check if the net intensity exceeds the threshold during gripping"""
        return net_intensity > ADAPTIVE_GRIPPING_CONFIG["threshold"]
