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
    
    # For step mode (diameter ≤1mm), return a default value
    if diameter_mm <= 1.0:
        return 0.3 * diameter_mm  # Estimate for step mode (will be calibrated in the future)
    
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


def calculate_expected_angle_from_motor_positions(m1_pos, m2_pos, object_diameter_mm):
    """
    Calculate the expected angle based on current motor positions.
    This represents the open-loop reference angle that the motors should achieve.
    
    Args:
        m1_pos: Current M1 motor position in steps
        m2_pos: Current M2 motor position in steps
        object_diameter_mm: Object diameter in millimeters
        
    Returns:
        float: Expected angle in degrees (0-360)
    """
    # Calculate the difference in motor positions
    # M1 and M2 move in opposite directions for rotation
    step_difference = m1_pos - m2_pos
    
    # For step mode (diameter ≤1mm), use a simple linear relationship
    if object_diameter_mm <= 1.0:
        # Use the estimated steps per degree for step mode
        steps_per_deg = get_steps_per_degree(object_diameter_mm, 'cw')  # Use CW as reference
        if steps_per_deg > 0:
            expected_angle = step_difference / (2 * steps_per_deg)  # Divide by 2 since both motors contribute
        else:
            expected_angle = 0.0
    else:
        # For angle mode, use the calibrated steps per degree
        steps_per_deg = get_steps_per_degree(object_diameter_mm, 'cw')  # Use CW as reference
        if steps_per_deg > 0:
            expected_angle = step_difference / (2 * steps_per_deg)  # Divide by 2 since both motors contribute
        else:
            expected_angle = 0.0
    
    # Normalize angle to 0-360 range
    expected_angle = expected_angle % 360
    if expected_angle < 0:
        expected_angle += 360
    
    return expected_angle


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
                                 initial_angle, target_angle, measured_angle, error]
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
                                'initial_angle', 'target_angle', 'measured_angle', 'error'])
            
            # Write data row
            writer.writerow(data_row)
            
        print(f"💾 Data point saved to CSV: diameter={data_row[0]}mm, target={data_row[5]}°, measured={data_row[6]:.1f}°, error={data_row[7]:.1f}°")
        
    except Exception as e:
        print(f"❌ Error saving CSV: {e}")


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
            
            # Add threshold line at 1 deg/s
            ax2.axhline(y=1.0, color='g', linestyle='--', alpha=0.7, label='1 deg/s threshold')
            ax2.axhline(y=-1.0, color='g', linestyle='--', alpha=0.7)
            
            # Label the low velocity point if found
            if low_velocity_point is not None and low_velocity_angle is not None:
                # Find the gradient value at this point
                low_velocity_grad = None
                for i, t in enumerate(gradient_times):
                    if t >= low_velocity_point:
                        low_velocity_grad = gradients[i]
                        break
                
                if low_velocity_grad is not None:
                    # Add annotation to velocity plot
                    ax2.annotate(f'Angle: {low_velocity_angle:.1f}°\nVel: {low_velocity_grad:.1f}°/s', 
                                xy=(low_velocity_point, low_velocity_grad),
                                xytext=(low_velocity_point + 0.5, low_velocity_grad + 5),
                                arrowprops=dict(arrowstyle='->', color='red'),
                                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
                                fontsize=10)
                    
                    # Add vertical line to angle plot
                    ax1.axvline(x=low_velocity_point, color='red', linestyle='--', alpha=0.7)
                    ax1.annotate(f'Low Vel Point\nAngle: {low_velocity_angle:.1f}°', 
                                xy=(low_velocity_point, low_velocity_angle),
                                xytext=(low_velocity_point + 0.5, low_velocity_angle + 30),
                                arrowprops=dict(arrowstyle='->', color='red'),
                                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
                                fontsize=10)
        
        plt.tight_layout()
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"📈 Encoder plot saved to: {plot_filename}")
        
        # Close the plot to prevent window from opening
        plt.close(fig)
        
    except Exception as e:
        print(f"❌ Error creating plot: {e}")


def calculate_low_velocity_point(data):
    """
    Calculate the first point where angular velocity drops below 1 deg/s.
    
    Args:
        data: List of encoder data points
        
    Returns:
        tuple: (low_velocity_time, low_velocity_angle) or (None, None) if not found
    """
    if len(data) < 2:
        return None, None
    
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
            if abs(gradient) < 1.0:  # First point below 1 deg/s
                return times[i], angles[i]
    
    return None, None
