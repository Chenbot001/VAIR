#!/usr/bin/env python3
"""
Test script for angle-based control functions
"""

# Add the same encoder calibration data for testing
ENCODER_CALIBRATION = {
    # Clockwise direction
    'cw': {
        2: 0.648711,  # steps per degree
        3: 0.995106,
        4: 1.276730,
        5: 1.643381
    },
    # Counter-clockwise direction  
    'ccw': {
        2: 0.646784,  # steps per degree
        3: 0.996116,
        4: 1.292792,
        5: 1.638945
    }
}

CONFIG = {
    "min_diameter_mm": 1,
    "max_diameter_mm": 5,
    "microsteps": 16,
}

def get_steps_per_degree(diameter_mm, direction):
    """
    Get steps per degree for given diameter and direction from calibration data.
    Uses interpolation for diameters between calibrated values.
    """
    diameter_mm = max(CONFIG["min_diameter_mm"], min(CONFIG["max_diameter_mm"], diameter_mm))
    
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
    """Convert angle in degrees to motor steps for given diameter and direction."""
    steps_per_deg = get_steps_per_degree(diameter_mm, direction)
    total_steps = angle_deg * steps_per_deg
    
    # Round to nearest 1/16 microstep
    microstep_resolution = 1.0 / CONFIG["microsteps"]
    rounded_steps = round(total_steps / microstep_resolution) * microstep_resolution
    
    return rounded_steps

def test_angle_conversions():
    """Test the angle conversion functions"""
    print("=" * 60)
    print("ANGLE CONVERSION TEST")
    print("=" * 60)
    
    # Test with different diameters and angles
    test_cases = [
        (2, 5, 'cw'),
        (3, 10, 'cw'),
        (4, 15, 'ccw'),
        (5, 20, 'ccw'),
        (2.5, 30, 'cw'),  # Test interpolation
        (3.7, 45, 'ccw'), # Test interpolation
    ]
    
    for diameter, angle, direction in test_cases:
        steps_per_deg = get_steps_per_degree(diameter, direction)
        steps = angle_to_steps(angle, diameter, direction)
        
        print(f"Diameter: {diameter}mm | Direction: {direction.upper()}")
        print(f"  Steps/degree: {steps_per_deg:.6f}")
        print(f"  {angle}° = {steps:.6f} steps")
        print(f"  Rounded to 1/16: {steps:.6f}")
        print()

if __name__ == "__main__":
    test_angle_conversions()
