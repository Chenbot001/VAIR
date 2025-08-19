# Sensor Image Display Integration Summary

## Overview
The gripper angle control script has been enhanced to display real-time sensor images including depth, raw image, deformation, and shear visualizations, based on the implementation from `main.py`.

## Changes Made

### 1. Updated SystemState Class
Added new fields to track sensor image display:
- `display_sensor_images`: Boolean flag to toggle image display on/off
- `sensor_fps`: Current sensor reading frame rate
- `frame_count`: Frame counter for FPS calculation
- `fps_start_time`: Start time for FPS calculation

### 2. New Functions Added

#### `display_sensor_images(sensor)`
- Displays a single OpenCV window showing combined sensor data:
  - **Depth**: Color-mapped depth data using COLORMAP_HOT
  - **Deformation**: 2D deformation vectors with arrows
  - **Shear**: Shear force vectors with arrows
  - All three images are concatenated horizontally with labels
- Calculates and updates sensor FPS
- Handles keyboard input ('r' key resets sensor from the window)

#### `close_sensor_windows()`
- Cleanly closes the sensor image window
- Used during shutdown and when toggling display off

### 3. Enhanced sensor_reader Function
- Now calls `display_sensor_images()` when image display is enabled
- Improved timing: 20Hz refresh rate when not gripping (up from 10Hz)
- Initializes FPS tracking
- Automatically closes windows when sensor thread exits

### 4. Updated User Interface

#### Display Dashboard
- Shows current image display status (ON/OFF)
- Displays sensor FPS when images are active
- Updated control descriptions

#### New Keyboard Control
- **[V]**: Toggle sensor image display on/off
- Updated controls documentation

#### Enhanced Cleanup
- Properly closes sensor image windows during program shutdown
- Ensures clean exit without orphaned windows

### 5. Updated Documentation
- Enhanced program header to include sensor image display feature
- Updated control descriptions to include new sensor functionality

## Usage Instructions

1. **Start the program**: Run `gripper_angle_control.py` as usual
2. **Enable image display**: Press `[V]` to toggle sensor image display on/off
3. **View sensor data**: A single window will appear showing:
   - Depth, deformation, and shear images side-by-side with labels
4. **Reset sensor**: Press `[R]` in the sensor window to reset the sensor
5. **Monitor performance**: Check the dashboard for sensor FPS

## Technical Details

### Image Processing
- Depth images use OpenCV's COLORMAP_HOT for visualization
- Deformation and shear vectors are scaled by 20x for better visibility
- Black background is used for vector displays
- All images update at the sensor reading frequency

### Performance
- Image display runs at 20Hz when not gripping
- Increases to 100Hz during active gripping operations
- FPS is calculated and displayed in real-time
- Minimal impact on existing gripper control functionality

### Integration
- Seamlessly integrates with existing depth intensity monitoring
- Does not interfere with adaptive gripping functionality
- All existing controls and features remain unchanged
- Optional feature that can be toggled on/off as needed

## Benefits

1. **Visual Feedback**: Real-time visualization of sensor data
2. **Debugging**: Easier troubleshooting of sensor issues
3. **Monitoring**: Visual confirmation of sensor operation
4. **Analysis**: Better understanding of contact forces and deformation
5. **Non-intrusive**: Optional feature that doesn't affect core functionality

The enhanced script now provides comprehensive sensor visualization while maintaining all existing gripper control capabilities.
