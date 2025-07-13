# Comprehensive Gripper Control System

This program integrates three components for precise and adaptive catheter and guidewire manipulation:

1. **Dynamixel trigger control** for gripper opening/closing
2. **Stepper motor control** for catheter rotation
3. **Daimon visuotactile sensor feedback** for adaptive grasping

## Hardware Setup

- **Gripper opening/closing**: Damiao servo motor (COM3)
- **Catheter rotation**: Linear travel stepper motors (rack and pinion setup, COM8)
- **Force feedback**: Daimon visuotactile sensors
- **Teleoperation**: Dynamixel motors from Gello device (isolated handle/trigger, COM4)

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Ensure all hardware modules are properly connected:
   - Dynamixel trigger motor on COM4
   - Damiao gripper motor on COM3
   - Stepper motor Arduino on COM8
   - Daimon visuotactile sensor

## Usage

Run the main program:
```bash
python gripper_complete.py
```

### Controls

- **Trigger position**: Controls gripper open/close
  - Above threshold (160°): Gripper opens
  - Below threshold: Gripper closes with adaptive feedback

- **Keyboard controls**:
  - `A`: Rotate catheter left
  - `D`: Rotate catheter right
  - `S`: Stop stepper motors
  - `R`: Reset stepper position
  - `ESC`: Quit program

- **Sensor feedback**: Automatically adapts gripper force based on object diameter

## Configuration

Edit the configuration section in `gripper_complete.py` to adjust:

- **Trigger settings**: ID, port, baud rate, threshold angle
- **Gripper settings**: Port, baud rate, position limits
- **Stepper settings**: Port, baud rate, step size, speed
- **Sensor settings**: Intensity threshold, display scales

## Features

### Adaptive Grasping
The system uses visuotactile sensor feedback to:
- Detect object contact
- Adapt gripper force based on object diameter
- Prevent over-compression of delicate objects

### Real-time Feedback
- Live display of gripper state and position
- Sensor intensity monitoring
- Trigger angle visualization
- Stepper motor status

### Safety Features
- Position limits for all motors
- Graceful shutdown on interrupt
- Error handling for hardware failures
- Automatic cleanup on exit

## Troubleshooting

### Import Errors
If you encounter import errors for local modules:
1. Ensure all subdirectories are present:
   - `dynamixel/libgx/`
   - `damiao/DM_Control/`
   - `daimon/`
2. Check that all required files exist in these directories
3. Verify Python path includes these directories

### Hardware Connection Issues
1. Check COM port assignments in Device Manager
2. Verify baud rates match hardware specifications
3. Ensure proper power supply to all components
4. Check USB connections and drivers

### Sensor Issues
1. Verify sensor is properly connected and powered
2. Check sensor ID (default: 0)
3. Ensure adequate lighting for sensor operation
4. Reset sensor if baseline readings are incorrect

## Architecture

The system uses a multi-threaded architecture:
- **Main control loop**: Handles trigger-based gripper control
- **Sensor monitoring thread**: Processes sensor data at 100Hz
- **Display thread**: Updates status display at 10Hz
- **Keyboard listener**: Handles user input

## File Structure

```
gripper_complete.py          # Main program
requirements.txt             # Python dependencies
README_gripper.md           # This documentation

dynamixel/                   # Dynamixel motor control
├── libgx/
│   ├── dynamixel_sdk.py
│   └── motor.py

damiao/                      # Damiao motor control
└── DM_Control/
    └── DM_CAN.py

daimon/                      # Daimon sensor control
├── dmrobotics/
│   └── __init__.py
└── dmSDK.py

stepper/                     # Stepper motor control
└── stepper_control.py
```

## Development

To extend the system:
1. Add new motor types in the appropriate subdirectory
2. Implement new control modes in the main controller
3. Add visualization features for new sensors
4. Extend configuration options as needed

## License

This project is part of a research system for multimodal intervention robotics. 