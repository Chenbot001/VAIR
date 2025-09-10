"""
Sensor Manager for the Gripper Control System
Manages visuotactile sensor interactions
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



# Add paths for local imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'daimon', 'dmrobotics'))

from config import (
    CONFIG, DISPLAY_CONFIG, ADAPTIVE_GRIPPING_CONFIG, MANUAL_TILT_CONFIG
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
        
        # Centerline detection variables
        self.centerline_points = None
        self.largest_contour = None
        self.angle_offset = None
        self.contact_threshold = 0.15

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
    
    def compute_centerline_angle(self, centerline_points):
        """
        Compute the angle offset of the centerline from vertical (0 degrees baseline).
        
        Args:
            centerline_points: Array of points along the centerline
            
        Returns:
            angle_offset: Absolute angle offset from vertical in degrees, or None if no centerline
        """
        if not NUMPY_AVAILABLE or centerline_points is None or len(centerline_points) < 2:
            return None
        
        # Use first and last points to determine the overall line direction
        start_point = centerline_points[0]
        end_point = centerline_points[-1]
        
        # Calculate direction vector
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        
        # Calculate angle from vertical (negative y-axis is up in image coordinates)
        # atan2 gives angle from positive x-axis, we want angle from negative y-axis (vertical up)
        angle_from_horizontal = np.arctan2(dy, dx)  # Angle from positive x-axis
        angle_from_vertical = angle_from_horizontal - np.pi/2  # Convert to angle from vertical
        
        # Convert to degrees
        angle_degrees = np.degrees(angle_from_vertical)
        
        # Normalize to [-90, 90] range and take absolute value
        # This ensures both CW and CCW deviations are positive
        if angle_degrees > 90:
            angle_degrees = 180 - angle_degrees
        elif angle_degrees < -90:
            angle_degrees = -180 - angle_degrees
        
        # Return absolute value for positive offset regardless of direction
        return abs(angle_degrees)
    
    def detect_centerline(self, depth_data):
        """
        Detect centerline from depth data by finding contours above threshold and fitting a line.
        
        Args:
            depth_data: The raw depth data array
            
        Returns:
            centerline_points: Array of points along the centerline, or None if no centerline found
            largest_contour: The largest contour found, or None
        """
        if not NUMPY_AVAILABLE or not CV2_AVAILABLE or depth_data is None:
            return None, None
        
        try:
            # Create binary mask for pixels above threshold
            contact_mask = (depth_data > self.contact_threshold).astype(np.uint8) * 255
            
            # Find contours
            contours, _ = cv2.findContours(contact_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
                return None, None
            
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Check if contour is large enough to be meaningful
            if cv2.contourArea(largest_contour) < 10:  # Minimum area threshold
                return None, None
            
            # Fit a line to the contour points
            # Reshape contour points for cv2.fitLine
            contour_points = largest_contour.reshape(-1, 2)
            
            if len(contour_points) < 5:  # Need minimum points for line fitting
                return None, None
            
            # Fit line using least squares
            [vx, vy, x0, y0] = cv2.fitLine(contour_points, cv2.DIST_L2, 0, 0.01, 0.01)
            
            # Generate centerline points along the fitted line
            # Get bounding box of the contour to determine line extent
            x_min, y_min, w, h = cv2.boundingRect(largest_contour)
            x_max = x_min + w
            y_max = y_min + h
            
            # Calculate line parameters
            if abs(vx) > 1e-6:  # Avoid division by zero
                # Calculate t values for line boundaries
                t1 = (x_min - x0) / vx
                t2 = (x_max - x0) / vx
                t_min = min(t1, t2)
                t_max = max(t1, t2)
                
                # Generate points along the line
                t_values = np.linspace(t_min, t_max, 50)
                centerline_points = np.column_stack([
                    x0 + t_values * vx,
                    y0 + t_values * vy
                ])
            else:
                # Vertical line case
                centerline_points = np.column_stack([
                    np.full(50, x0),
                    np.linspace(y_min, y_max, 50)
                ])
            
            # Filter points to stay within image bounds
            h_img, w_img = depth_data.shape
            valid_mask = (
                (centerline_points[:, 0] >= 0) & 
                (centerline_points[:, 0] < w_img) &
                (centerline_points[:, 1] >= 0) & 
                (centerline_points[:, 1] < h_img)
            )
            centerline_points = centerline_points[valid_mask]
            
            if len(centerline_points) < 2:
                return None, None
            
            return centerline_points.astype(int), largest_contour
            
        except Exception as e:
            self.last_message = f"ERR: Centerline detection failed: {e}"
            return None, None
    
    def add_centerline_overlay(self, depth_img, centerline_points, largest_contour=None):
        """
        Add centerline and contour overlay to the depth image.
        
        Args:
            depth_img: The depth image (BGR format)
            centerline_points: Array of points along the centerline
            largest_contour: The contour from which centerline was derived
            
        Returns:
            The depth image with centerline overlaid
        """
        if not CV2_AVAILABLE or depth_img is None:
            return depth_img
        
        depth_img_with_centerline = depth_img.copy()
        
        if centerline_points is not None and len(centerline_points) > 1:
            # Draw the centerline (50% thinner: 3 -> 1.5, rounded to 2)
            for i in range(len(centerline_points) - 1):
                pt1 = tuple(centerline_points[i])
                pt2 = tuple(centerline_points[i + 1])
                cv2.line(depth_img_with_centerline, pt1, pt2, (0, 255, 0), 2)  # Green line
            
            # Optionally draw the contour outline (50% thinner: 2 -> 1)
            if largest_contour is not None:
                cv2.drawContours(depth_img_with_centerline, [largest_contour], -1, (0, 255, 255), 1)  # Yellow contour
        
        return depth_img_with_centerline
    
    def add_angle_offset_overlay(self, depth_img, angle_offset):
        """
        Add angle offset value overlay to the depth image.
        
        Args:
            depth_img: The colorized depth image (BGR format)
            angle_offset: The angle offset from vertical in degrees, or None if no centerline
            
        Returns:
            The depth image with angle offset value overlaid
        """
        if not CV2_AVAILABLE or depth_img is None:
            return depth_img
        
        # Create a copy of the depth image to avoid modifying the original
        depth_img_with_overlay = depth_img.copy()
        
        # Prepare the text to display
        if angle_offset is not None:
            # Round to nearest 10 degrees for better readability - remove for exact angle
            rounded_angle = round(angle_offset / 10) * 10
            angle_text = f"Angle Offset: {rounded_angle:.0f} degrees"
        else:
            angle_text = "Angle Offset: N/A"
        
        # Set text properties
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        color = (255, 255, 255)  # White color in BGR
        thickness = 1
        
        # Get text size to position it properly
        (text_width, text_height), baseline = cv2.getTextSize(angle_text, font, font_scale, thickness)
        
        # Position the text in the top-left corner with some padding
        x = 10
        y = text_height + 15
        
        # Add a semi-transparent background rectangle for better readability
        overlay = depth_img_with_overlay.copy()
        cv2.rectangle(overlay, (x-5, y-text_height-5), (x+text_width+5, y+baseline+5), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, depth_img_with_overlay, 0.4, 0, depth_img_with_overlay)
        
        # Add the text
        cv2.putText(depth_img_with_overlay, angle_text, (x, y), font, font_scale, color, thickness)
        
        return depth_img_with_overlay
    
    def set_contact_threshold(self, threshold):
        """
        Set the contact threshold for centerline detection.
        
        Args:
            threshold: New threshold value (default: 0.15)
        """
        self.contact_threshold = threshold
        self.last_message = f"Contact threshold set to {threshold:.3f}"
    
    def get_centerline_info(self):
        """
        Get current centerline detection information.
        
        Returns:
            dict: Information about current centerline detection
        """
        return {
            'centerline_detected': self.centerline_points is not None,
            'angle_offset': self.angle_offset,
            'contact_threshold': self.contact_threshold,
            'contour_area': cv2.contourArea(self.largest_contour) if self.largest_contour is not None and CV2_AVAILABLE else 0,
            'centerline_points_count': len(self.centerline_points) if self.centerline_points is not None else 0
        }
    
    def get_rounded_tilt_angle(self):
        """
        Get the rounded tilt angle (angle offset from vertical) for recording purposes.
        
        Returns:
            float: Rounded tilt angle in degrees, or 0.0 if no angle is available
        """
        # Check if manual tilt mode is enabled
        if MANUAL_TILT_CONFIG.get("enabled", False):
            return float(MANUAL_TILT_CONFIG.get("current_value", 0))
        
        # Otherwise use calculated centerline angle
        if self.angle_offset is not None:
            return round(self.angle_offset / 10) * 10
        return 0.0
    
    def display_sensor_images(self):
        """Display concatenated sensor images with centerline detection"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return
        
        if not CV2_AVAILABLE or not NUMPY_AVAILABLE:
            return
        
        # Check if sensor image display is enabled in configuration
        if not DISPLAY_CONFIG.get("show_sensor_images", True):
            return
        
        try:
            # Get sensor data
            img = self.sensor.getRawImage()
            depth = self.sensor.getDepth()
            deformation = self.sensor.getDeformation2D()
            shear = self.sensor.getShear()
            
            # Create depth image with color mapping
            depth_img = cv2.applyColorMap((depth * 0.25 * 255.0).astype('uint8'), cv2.COLORMAP_HOT)
            
            # Detect centerline from depth data
            self.centerline_points, self.largest_contour = self.detect_centerline(depth)
            
            # Compute angle offset from vertical
            self.angle_offset = self.compute_centerline_angle(self.centerline_points)
            
            # Add centerline overlay if detected
            if self.centerline_points is not None:
                depth_img = self.add_centerline_overlay(depth_img, self.centerline_points, self.largest_contour)
            
            # Add angle offset overlay to the depth image
            depth_img = self.add_angle_offset_overlay(depth_img, self.angle_offset)
            
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
            font_scale = 0.5
            font_color = (255, 255, 255)
            thickness = 1
            
            cv2.putText(depth_img, 'Depth', (10, height-10), font, font_scale, font_color, thickness)
            cv2.putText(deformation_img, 'Deformation', (10, height-10), font, font_scale, font_color, thickness)
            cv2.putText(shear_img, 'Shear', (10, height-10), font, font_scale, font_color, thickness)
            
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
        # Calculate sleep time based on configured update rate
        sensor_update_interval = 1.0 / DISPLAY_CONFIG.get("sensor_image_update_rate_hz", 10)
        
        while state.running:
            try:
                if self.sensor and self.connected:
                    self.get_max_depth_intensity()
                    self.display_sensor_images()
                    
                    # Read more frequently when gripping is active
                    if hasattr(state, 'hardware') and hasattr(state.hardware, 'gripper') and state.hardware.gripper.is_gripping:
                        time.sleep(0.01)  # 100Hz when gripping
                    else:
                        time.sleep(sensor_update_interval)  # Use configured update rate
            except Exception as e:
                self.last_message = f"ERR: Sensor reading failed: {e}"
                self.connected = False
                break
        
        self.close_sensor_windows()

    def get_latest_deformation(self):
        """Get the latest deformation data from the sensor"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return None
        
        try:
            return self.sensor.getDeformation2D()
        except Exception as e:
            self.last_message = f"ERR: Failed to get deformation data: {e}"
            return None

    def get_latest_shear(self):
        """Get the latest shear data from the sensor"""
        if not self.sensor or not SENSOR_AVAILABLE:
            return None
        
        try:
            return self.sensor.getShear()
        except Exception as e:
            self.last_message = f"ERR: Failed to get shear data: {e}"
            return None


class SensorManager:
    """Main sensor manager that coordinates visuotactile sensor"""
    
    def __init__(self):
        self.visuotactile = DaimonManager()
        self.available = {
            'visuotactile': SENSOR_AVAILABLE,
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
        
        return self.available
    
    def cleanup(self):
        """Clean up sensor connections"""
        print("Cleaning up sensor connections...")
        self.visuotactile.disconnect()
    
    def get_status(self):
        """Get status of visuotactile sensor"""
        return {
            'visuotactile': {
                'connected': self.visuotactile.connected,
                'last_message': self.visuotactile.last_message,
                'max_intensity': self.visuotactile.max_depth_intensity,
                'baseline': self.visuotactile.baseline_intensity,
                'net_intensity': self.visuotactile.net_intensity,
                'sensor_fps': self.visuotactile.sensor_fps,
                'centerline_detected': self.visuotactile.centerline_points is not None,
                'angle_offset': self.visuotactile.angle_offset,
                'contact_threshold': self.visuotactile.contact_threshold
            }
        }
    
    def check_gripping_condition(self, net_intensity):
        """Check if the net intensity exceeds the threshold during gripping"""
        return net_intensity > ADAPTIVE_GRIPPING_CONFIG["threshold"]
    
    def toggle_manual_tilt_mode(self):
        """Toggle between manual tilt mode and automatic centerline calculation"""
        MANUAL_TILT_CONFIG["enabled"] = not MANUAL_TILT_CONFIG["enabled"]
        mode = "Manual" if MANUAL_TILT_CONFIG["enabled"] else "Automatic (Centerline)"
        current_value = MANUAL_TILT_CONFIG["current_value"]
        print(f"Tilt mode: {mode} | Current value: {current_value}°")
        return MANUAL_TILT_CONFIG["enabled"]
    
    def increment_manual_tilt(self):
        """Increment manual tilt to next valid value"""
        if not MANUAL_TILT_CONFIG["enabled"]:
            return MANUAL_TILT_CONFIG["current_value"]
        
        valid_values = MANUAL_TILT_CONFIG["valid_values"]
        current_value = MANUAL_TILT_CONFIG["current_value"]
        
        try:
            current_index = valid_values.index(current_value)
            next_index = (current_index + 1) % len(valid_values)
            MANUAL_TILT_CONFIG["current_value"] = valid_values[next_index]
        except ValueError:
            # If current value is not in valid values, reset to first
            MANUAL_TILT_CONFIG["current_value"] = valid_values[0]
        
        print(f"Manual tilt set to: {MANUAL_TILT_CONFIG['current_value']}°")
        return MANUAL_TILT_CONFIG["current_value"]
    
    def decrement_manual_tilt(self):
        """Decrement manual tilt to previous valid value"""
        if not MANUAL_TILT_CONFIG["enabled"]:
            return MANUAL_TILT_CONFIG["current_value"]
        
        valid_values = MANUAL_TILT_CONFIG["valid_values"]
        current_value = MANUAL_TILT_CONFIG["current_value"]
        
        try:
            current_index = valid_values.index(current_value)
            prev_index = (current_index - 1) % len(valid_values)
            MANUAL_TILT_CONFIG["current_value"] = valid_values[prev_index]
        except ValueError:
            # If current value is not in valid values, reset to last
            MANUAL_TILT_CONFIG["current_value"] = valid_values[-1]
        
        print(f"Manual tilt set to: {MANUAL_TILT_CONFIG['current_value']}°")
        return MANUAL_TILT_CONFIG["current_value"]
