"""
Utility functions for the Gripper Control System
Contains helper functions for angle conversion, data processing, and other utilities
"""

import os
import csv
import numpy as np
from datetime import datetime
from config import ENCODER_CALIBRATION, CONFIG, PATHS

# Try to import matplotlib, but don't fail if it's not available
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available. Plotting functions will be disabled.")


def get_steps_per_degree(diameter_mm, direction):
    """
    Get steps per degree for given diameter and direction from calibration data.
    Uses interpolation for diameters between calibrated values.
    
    Args:
        diameter_mm: Object diameter in millimeters (1-5)
        direction: 'cw' or 'ccw'
        
    Returns:
        float: Steps per degree
    """
    diameter_mm = max(CONFIG["min_diameter_mm"], min(CONFIG["max_diameter_mm"], diameter_mm))
    
    # For step mode (diameter = 1mm), return a default value
    if diameter_mm == 1.0:
        return 1.0  # 1 step per degree for step mode (will be calibrated in the future)
    
    calibration = ENCODER_CALIBRATION[direction]
    
    # If exact diameter exists, return it
    if diameter_mm in calibration:
        return calibration[diameter_mm]
    
    # Otherwise, interpolate between nearest values
    diameters = sorted(calibration.keys())
    
    # Find the two closest diameters
    lower_d = max([d for d in diameters if d <= diameter_mm])
    upper_d = min([d for d in diameters if d >= diameter_mm])
    
    if lower_d == upper_d:
        return calibration[lower_d]
    
    # Linear interpolation
    lower_steps = calibration[lower_d]
    upper_steps = calibration[upper_d]
    
    ratio = (diameter_mm - lower_d) / (upper_d - lower_d)
    interpolated_steps = lower_steps + ratio * (upper_steps - lower_steps)
    
    return interpolated_steps


def angle_to_steps(angle_deg, diameter_mm, direction):
    """
    Convert angle in degrees to motor steps for given diameter and direction.
    
    Args:
        angle_deg: Angle in degrees
        diameter_mm: Object diameter in millimeters
        direction: 'cw' or 'ccw'
        
    Returns:
        float: Number of steps (will be rounded to 1/16 microstep)
    """
    steps_per_deg = get_steps_per_degree(diameter_mm, direction)
    total_steps = angle_deg * steps_per_deg
    
    # Round to nearest 1/16 microstep
    microstep_resolution = 1.0 / CONFIG["microsteps"]
    rounded_steps = round(total_steps / microstep_resolution) * microstep_resolution
    
    return rounded_steps


def steps_to_angle(steps, diameter_mm, direction):
    """
    Convert motor steps to angle in degrees for given diameter and direction.
    
    Args:
        steps: Number of steps
        diameter_mm: Object diameter in millimeters
        direction: 'cw' or 'ccw'
        
    Returns:
        float: Angle in degrees
    """
    steps_per_deg = get_steps_per_degree(diameter_mm, direction)
    if steps_per_deg == 0:
        return 0.0
    return steps / steps_per_deg


def calculate_motor_positions(target_angle_deg, direction, current_m1_pos, current_m2_pos, object_diameter_mm):
    """
    Calculate motor positions for a given target angle and direction.
    
    Args:
        target_angle_deg: Target angle in degrees
        direction: 'cw' or 'ccw'
        current_m1_pos: Current M1 motor position
        current_m2_pos: Current M2 motor position
        object_diameter_mm: Object diameter in millimeters
        
    Returns:
        tuple: (m1_pos, m2_pos) - new motor positions
    """
    # Get the step increment for this angle and direction
    step_increment = angle_to_steps(abs(target_angle_deg), object_diameter_mm, direction)
    
    # Calculate new positions based on direction
    if direction == 'ccw':  # A key - counter-clockwise
        new_m1 = current_m1_pos + step_increment
        new_m2 = current_m2_pos - step_increment
    else:  # direction == 'cw' - D key - clockwise
        new_m1 = current_m1_pos - step_increment
        new_m2 = current_m2_pos + step_increment
    
    # Clamp to physical limits
    new_m1 = max(0, min(CONFIG["max_steps"], new_m1))
    new_m2 = max(0, min(CONFIG["max_steps"], new_m2))
    
    return new_m1, new_m2


def unwrap_angles(angles):
    """
    Unwrap angle data to handle 0°/360° transitions smoothly.
    
    Args:
        angles (list): List of angle values in degrees
    
    Returns:
        list: Unwrapped angle values (continuous, may exceed 360°)
    """
    if not angles:
        return angles
    
    unwrapped = [angles[0]]  # Start with the first angle
    
    for i in range(1, len(angles)):
        prev_angle = unwrapped[i-1]
        current_angle = angles[i]
        
        # Calculate the difference
        diff = current_angle - (prev_angle % 360)
        
        # Handle wrap-around cases
        if diff > 180:  # Wrapped from ~360° to ~0°
            adjustment = -360
        elif diff < -180:  # Wrapped from ~0° to ~360°
            adjustment = 360
        else:
            adjustment = 0
        
        # Add the adjusted angle
        unwrapped.append(prev_angle + diff + adjustment)
    
    return unwrapped


def percentage_to_position(percentage):
    """Convert closure percentage to radian position"""
    from config import GRIPPER_MAX_POS, GRIPPER_MIN_POS
    
    percentage = max(0, min(100, percentage))
    position = GRIPPER_MAX_POS + (percentage/100.0) * (GRIPPER_MIN_POS - GRIPPER_MAX_POS)
    return position


def position_to_percentage(position):
    """Convert radian position to closure percentage"""
    from config import GRIPPER_MAX_POS, GRIPPER_MIN_POS
    
    position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
    percentage = ((position - GRIPPER_MAX_POS) / (GRIPPER_MIN_POS - GRIPPER_MAX_POS)) * 100
    return percentage


def safe_gripper_position(position):
    """Ensures the gripper position stays within safe limits"""
    from config import GRIPPER_MIN_POS, GRIPPER_MAX_POS
    
    limited_position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
    return limited_position


def ensure_directories():
    """Ensure all required directories exist"""
    os.makedirs(PATHS["encoder_data_dir"], exist_ok=True)
    os.makedirs(PATHS["encoder_plot_dir"], exist_ok=True)


def save_encoder_data_to_csv(data_row):
    """
    Save encoder data to CSV file.
    
    Args:
        data_row: List containing [diameter, grip_strength, direction, steps, 
                                 initial_angle, target_angle, measured_angle, error, tilt]
    """
    ensure_directories()
    
    # Check if CSV file exists to determine if we need to write headers
    file_exists = os.path.exists(PATHS["encoder_csv_file"])
    
    try:
        with open(PATHS["encoder_csv_file"], 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write headers if file doesn't exist
            if not file_exists:
                writer.writerow(['diameter', 'grip_strength', 'direction', 'steps', 
                                'initial_angle', 'target_angle', 'measured_angle', 'error', 'tilt'])
            
            # Write data row
            writer.writerow(data_row)
            
        print(f"💾 Data point saved to CSV: diameter={data_row[0]}mm, target={data_row[5]}°, measured={data_row[6]:.1f}°, error={data_row[7]:.1f}°, tilt={data_row[8]:.1f}°")
        
    except Exception as e:
        print(f"❌ Error saving CSV: {e}")


def save_encoder_thin_data_to_csv(recorded_data, session_metadata):
    """
    Save time series encoder data for 1mm diameter (thin wire) to a separate CSV file.
    
    Args:
        recorded_data: List of time series data points from recording session
        session_metadata: Dict containing session information (diameter, target, direction, etc.)
    """
    ensure_directories()
    
    # Create the thin data CSV file path
    thin_csv_file = os.path.join(PATHS["encoder_data_dir"], "encoder_data_thin.csv")
    
    # Check if CSV file exists to determine if we need to write headers
    file_exists = os.path.exists(thin_csv_file)
    
    # Generate a unique session ID based on timestamp
    session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    try:
        with open(thin_csv_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write headers if file doesn't exist
            if not file_exists:
                writer.writerow([
                    'session_id', 'diameter', 'target_steps', 'direction', 'grip_strength', 
                    'initial_angle', 'timestamp', 'elapsed_time', 'angle'
                ])
            
            # Write all time series data points for this session
            for data_point in recorded_data:
                # data_point format: [timestamp, elapsed_time, angle, turn_count, single_turn_value, total_encoded_value, direction_str]
                row = [
                    session_id,
                    session_metadata.get('diameter', 1.0),
                    session_metadata.get('target_steps', 0),
                    session_metadata.get('direction', 'unknown'),
                    session_metadata.get('grip_strength', 0.0),
                    session_metadata.get('initial_angle', 0.0),
                    data_point[0],  # timestamp
                    data_point[1],  # elapsed_time
                    data_point[2]   # angle
                ]
                writer.writerow(row)
            
        print(f"💾 Thin wire session data saved: {len(recorded_data)} points to encoder_data_thin.csv (Session: {session_id})")
        
    except Exception as e:
        print(f"❌ Error saving thin wire CSV: {e}")


def create_encoder_plot(data, duration, diameter, target_angle, direction, low_velocity_point=None, low_velocity_angle=None):
    """
    Create and save encoder data plots.
    
    Args:
        data: List of [timestamp, elapsed_time, angle, turn_count, single_turn_value, total_encoded_value, direction_str]
        duration: Recording duration in seconds
        diameter: Object diameter
        target_angle: Target angle
        direction: Rotation direction
        low_velocity_point: Time point where velocity drops below threshold
        low_velocity_angle: Angle at low velocity point
    """
    if not MATPLOTLIB_AVAILABLE:
        print("⚠ Plotting disabled: matplotlib not available")
        return
        
    if not data:
        print("No encoder data to plot")
        return
    
    try:
        # Extract data for plotting
        times = [row[1] for row in data]
        angles = [row[2] for row in data]
        
        # Unwrap angles to handle 0°/360° transitions
        unwrapped_angles = unwrap_angles(angles)
        
        # Calculate gradient (angular velocity) using unwrapped angles
        gradients = []
        gradient_times = []
        
        for i in range(1, len(unwrapped_angles)):
            dt = times[i] - times[i-1]
            dangle = unwrapped_angles[i] - unwrapped_angles[i-1]
            if dt > 0:  # Avoid division by zero
                gradient = dangle / dt  # degrees per second
                gradients.append(gradient)
                gradient_times.append(times[i])
        
        # Create filename
        plot_filename = f"{PATHS['encoder_plot_dir']}/plot_{diameter}_{target_angle}_{direction}.png"
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        fig.suptitle(f'Rotary Encoder Data - {duration:.2f}s Recording', fontsize=16)
        
        # Plot 1: Original angle over time (0-360°)
        ax1.plot(times, angles, 'b-', linewidth=1.5)
        ax1.set_ylabel('Angle (degrees)')
        ax1.set_title('Angle vs Time')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Gradient (angular velocity) over time (using unwrapped angles)
        if gradients:
            ax2.plot(gradient_times, gradients, 'r-', linewidth=1.5)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angular Velocity (deg/s)')
            ax2.set_title('Angular Velocity vs Time (Smooth)')
            ax2.grid(True, alpha=0.3)
            ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)  # Zero line
            
            # Add threshold line based on diameter
            if diameter <= 1.0:
                threshold_value = 0.5
                threshold_label = '0.5 deg/s threshold (1mm)'
            else:
                threshold_value = 1.0
                threshold_label = '1 deg/s threshold (2-5mm)'
            
            ax2.axhline(y=threshold_value, color='g', linestyle='--', alpha=0.7, label=threshold_label)
            ax2.axhline(y=-threshold_value, color='g', linestyle='--', alpha=0.7)
            
            # Label the low velocity point if found
            if low_velocity_point is not None and low_velocity_angle is not None:
                # Find the gradient value at this point
                low_velocity_grad = None
                for i, t in enumerate(gradient_times):
                    if t >= low_velocity_point:
                        low_velocity_grad = gradients[i]
                        break
                
                if low_velocity_grad is not None:
                    # Add visible dots at the low velocity points
                    ax1.plot(low_velocity_point, low_velocity_angle, 'ro', markersize=8, markerfacecolor='red', markeredgecolor='darkred', markeredgewidth=2, label='Low Velocity Point')
                    ax2.plot(low_velocity_point, low_velocity_grad, 'ro', markersize=8, markerfacecolor='red', markeredgecolor='darkred', markeredgewidth=2, label='Low Velocity Point')
                    
                    # Add vertical line to angle plot for reference
                    ax1.axvline(x=low_velocity_point, color='red', linestyle='--', alpha=0.3)
                    ax2.axvline(x=low_velocity_point, color='red', linestyle='--', alpha=0.3)
                    
                    # Position annotations in appropriate corners without arrows
                    # For angle plot: place in upper right corner
                    ax1.text(0.98, 0.98, f'Low Vel Point\nAngle: {low_velocity_angle:.1f}°', 
                            transform=ax1.transAxes, 
                            verticalalignment='top', horizontalalignment='right',
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8),
                            fontsize=10)
                    
                    # For velocity plot: place in upper right corner
                    ax2.text(0.98, 0.98, f'Angle: {low_velocity_angle:.1f}°\nVel: {low_velocity_grad:.1f}°/s', 
                            transform=ax2.transAxes,
                            verticalalignment='top', horizontalalignment='right',
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8),
                            fontsize=10)
        
        plt.tight_layout()
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"📈 Encoder plot saved to: {plot_filename}")
        
        # Close the plot to prevent window from opening
        plt.close(fig)
        
    except Exception as e:
        print(f"❌ Error creating plot: {e}")


def calculate_low_velocity_point(data, diameter_mm=None):
    """
    Calculate the first point where angular velocity drops below threshold.
    Uses different thresholds based on diameter:
    - 1mm: 0.5 deg/s threshold (more sensitive for step-based control)
    - 2-5mm: 1.0 deg/s threshold (standard for angle-based control)
    
    Args:
        data: List of encoder data points
        diameter_mm: Wire diameter in mm (optional, defaults to 1.0 deg/s threshold)
        
    Returns:
        tuple: (low_velocity_time, low_velocity_angle) or (None, None) if not found
    """
    if len(data) < 2:
        return None, None
    
    # Set velocity threshold based on diameter
    if diameter_mm is not None and diameter_mm <= 1.0:
        velocity_threshold = 0.5  # Lower threshold for 1mm case
    else:
        velocity_threshold = 1.0  # Standard threshold for 2-5mm case
    
    # Extract data
    times = [row[1] for row in data]
    angles = [row[2] for row in data]
    
    # Unwrap angles
    unwrapped_angles = unwrap_angles(angles)
    
    # Calculate gradients
    for i in range(1, len(unwrapped_angles)):
        dt = times[i] - times[i-1]
        dangle = unwrapped_angles[i] - unwrapped_angles[i-1]
        if dt > 0:
            gradient = dangle / dt
            if abs(gradient) < velocity_threshold:  # First point below threshold
                return times[i], angles[i]
    
    return None, None


def calculate_angular_displacement(start_angle, end_angle, direction):
    """
    Calculate angular displacement between two angles for a FIXED CW ENCODER.
    
    IMPORTANT: This encoder is fixed to CW direction and always increases clockwise.
    - CW motor movement: encoder increases normally
    - CCW motor movement: encoder still increases, showing complement angle
    
    Args:
        start_angle: Starting angle in degrees (0-360)
        end_angle: Ending angle in degrees (0-360)  
        direction: Expected movement direction ('cw' or 'ccw')
        
    Returns:
        float: Angular displacement in degrees (always positive)
    """
    # Calculate raw difference (how much encoder increased)
    diff = end_angle - start_angle
    
    # Handle 0°/360° wrapping: if diff is negative, we crossed the boundary
    if diff < 0:
        diff += 360.0
    
    # Now diff represents how much the encoder increased (0-360°)
    
    if direction == 'cw':
        # CW motor movement: encoder increase directly represents rotation magnitude
        displacement = diff
    else:  # ccw
        # CCW motor movement: encoder increase represents complement of actual rotation
        # For small CCW rotations: encoder shows small increase (actual displacement)
        # For large CCW rotations: encoder shows large increase (complement of displacement)
        # We need to determine which case this is:
        
        if diff <= 180.0:
            # Small encoder increase likely means small CCW rotation
            displacement = diff
        else:
            # Large encoder increase likely means large CCW rotation
            # The actual CCW displacement is the complement
            displacement = 360.0 - diff
    
    return abs(displacement)


def save_encoder_steps_data_to_csv(diameter_mm, speed, target_steps, angle, direction, tilt):
    """
    Save step-based encoder data to the specific steps CSV file.
    
    Args:
        diameter_mm: Wire diameter in mm
        speed: Motor speed in steps/sec
        target_steps: Target steps for movement
        angle: Measured angular displacement (non-negative)
        direction: Movement direction ('cw' or 'ccw')
        tilt: Tilt angle value from visuotactile sensor
    """
    # Use the same file as main_thin.py for consistency
    csv_file_path = os.path.join("gripper_test", "encoder_data_steps.csv")
    
    # Ensure directory exists
    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    
    # Check if file exists to determine if we need headers
    file_exists = os.path.exists(csv_file_path)
    
    try:
        with open(csv_file_path, 'a', newline='') as csvfile:
            fieldnames = ['diameter', 'speed', 'step', 'measured_angle', 'direction', 'tilt']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            # Write header if file is new
            if not file_exists:
                writer.writeheader()
            
            # Write data row
            writer.writerow({
                'diameter': diameter_mm,
                'speed': speed,
                'step': target_steps,
                'measured_angle': f"{abs(angle):.2f}",  # Ensure non-negative angle
                'direction': direction,
                'tilt': tilt
            })
            
        print(f"💾 Step-based data saved: {diameter_mm}mm | {target_steps} steps | {abs(angle):.2f}° | {direction.upper()}")
        
    except Exception as e:
        print(f"❌ Error saving step-based data to CSV: {e}")
