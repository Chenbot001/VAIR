# Dynamixel Motor Control System

This project provides a comprehensive motor control system for robotic applications using Dynamixel motors and DM-Tac grippers. It includes various teleoperation modes and motor control utilities.

## Features

The system includes several control modes and utilities:

### 1. Keyboard Teleoperation (`keyboard_teleop.py`)
- **G key**: Switch to GRASP mode - closes the gripper with sensor feedback
- **R key**: Switch to RELEASE mode - opens the gripper fully
- **Q key**: Quit the program
- Includes real-time sensor feedback monitoring
- Displays gripper position and sensor intensity

### 2. Trigger-based Teleoperation with Feedback (`trigger_teleop_fb.py`)
- Uses a Dynamixel motor as a trigger input (ID: 7, COM4)
- Automatically controls gripper based on trigger angle threshold (160°)
- Includes sensor feedback for safe grasping
- Real-time monitoring of trigger angle, gripper state, and sensor data

### 3. Trigger-based Teleoperation without Feedback (`trigger_teleop_no_fb.py`)
- Similar to the feedback version but without sensor monitoring
- Simpler control logic for basic trigger-to-gripper mapping
- Lower threshold angle (160°) for grasp activation

### 4. Motor Angle Reader (`read_motor_angle.py`)
- Simple utility to read and display motor position in real-time
- Useful for calibration and testing
- Displays angle in degrees with continuous updates

### 5. Advanced Motor Control (`gripper_teleop/motor_control.py`)
- Complete 7-DOF robotic arm control system
- Supports both left and right arm configurations
- Includes joint limit safety checks
- Position and velocity control for all joints
- Integration with PyBullet for simulation

## System Requirements

- Python 3.7 or higher
- Windows, macOS, or Linux with available COM ports
- Dynamixel motors and DM-Tac gripper hardware
- Required Python packages (see requirements.txt)

## Dependencies

```bash
numpy>=1.20.0
pyserial>=3.5
keyboard>=0.13.5
```

Additional dependencies for advanced features:
- `pybullet` (for simulation)
- `ikpy` (for inverse kinematics)
- Custom DM-Tac SDK modules

## Hardware Setup

### Basic Setup
1. Connect Dynamixel motors to COM4 (default)
2. Connect DM-Tac gripper to COM3 (default)
3. Ensure proper power supply and communication

### Advanced Setup (7-DOF Arm)
- Configure motor IDs according to left/right arm specification
- Set appropriate joint limits for your hardware
- Verify all motor connections and power requirements

## Installation

### 1. Environment Setup
```bash
# Create and activate virtual environment (recommended)
python -m venv env
# Windows
.\env\Scripts\activate
# macOS/Linux
source env/bin/activate
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt
```

### 3. Additional Setup
Ensure the following directories are in your Python path:
- `../damiao/DM_Control` (for DM-Tac SDK)
- `../daimon` (for sensor modules)
- `libgx/` (for Dynamixel SDK)

## Usage

### Basic Motor Control
```bash
# Read motor angle
python read_motor_angle.py

# Keyboard teleoperation
python keyboard_teleop.py

# Trigger-based control with feedback
python trigger_teleop_fb.py

# Trigger-based control without feedback
python trigger_teleop_no_fb.py
```

### Advanced Arm Control
```python
from gripper_teleop.motor_control import MotorController

# Initialize controller
controller = MotorController()
controller.Motor_Init('COM3', 921600, 'left')  # or 'right'

# Control all joints
positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocities = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
controller.Motor_all_control(positions, velocities)
```

## Configuration

### Port Settings
- **Dynamixel Motors**: COM4 (default), 1M baud rate
- **DM-Tac Gripper**: COM3 (default), 921600 baud rate
- **Motor ID**: 7 (for trigger motor)

### Threshold Settings
- **Trigger Threshold**: 160° (configurable in scripts)
- **Sensor Intensity Threshold**: 0.1 (for feedback control)
- **Gripper Position Limits**: 0.0 to -1.38 radians

## Safety Features

- Joint limit enforcement for 7-DOF arms
- Sensor feedback for safe grasping
- Emergency stop functionality
- Position and velocity constraints
- Automatic motor disable on program exit

## Troubleshooting

### Common Issues

1. **Port Connection Errors**
   - Verify COM port numbers in device manager
   - Check cable connections
   - Ensure proper permissions (Linux/macOS may need `sudo chmod 666 /dev/ttyUSB0`)

2. **Motor Communication Issues**
   - Verify motor IDs match configuration
   - Check baud rate settings
   - Ensure motors are powered and enabled

3. **Import Errors**
   - Verify all required directories are in Python path
   - Check that DM-Tac SDK is properly installed
   - Ensure all dependencies are installed

4. **Keyboard Input Issues**
   - In remote environments, keyboard input may not work as expected
   - Consider using trigger-based control instead

### Debugging
- Check `sdk_log.log` for Dynamixel SDK messages
- Monitor serial communication with appropriate tools
- Use `read_motor_angle.py` to verify basic motor communication

## File Structure

```
dynamixel/
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── keyboard_teleop.py        # Keyboard-based gripper control
├── trigger_teleop_fb.py      # Trigger control with sensor feedback
├── trigger_teleop_no_fb.py   # Trigger control without feedback
├── read_motor_angle.py       # Motor position reader utility
├── libgx/                    # Dynamixel SDK wrapper
├── gripper_teleop/           # Advanced motor control
│   ├── motor_control.py      # 7-DOF arm controller
│   └── trigger_control.py    # Trigger control utilities
└── sdk_log.log              # SDK operation log
```

## Contributing

When adding new features:
1. Follow the existing code structure
2. Include proper error handling
3. Add safety checks for motor operations
4. Update this README with new functionality
5. Test thoroughly with actual hardware

## License

This project is part of the Multimodal Intervention Robot system. Please refer to the main project documentation for licensing information.
