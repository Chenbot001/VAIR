# Gripper Control System - Wire Diameter Adaptive Control

This system provides precision gripper control with two specialized modes optimized for different wire diameters. The system automatically adapts its control strategy based on the target wire diameter, using angle-based control for thick wires and step-based control for thin wires.

## System Overview

The gripper control system supports two distinct operating modes:

### **Thick Wire Mode (2-5mm diameter)**
- **Control Method**: Angle-based positioning
- **Target Application**: Thick wires, cables, and rigid objects
- **Precision**: High-accuracy angular positioning using encoder feedback
- **Main Application**: `main_encoder.py`

### **Thin Wire Mode (≤1mm diameter)**  
- **Control Method**: Step-based positioning
- **Target Application**: Thin wires (0.2mm, 0.5mm, 1.0mm)
- **Precision**: Fine-grained step control for delicate manipulation
- **Main Application**: `main_thin.py`

## Architecture Overview

```
gripper_complete/
├── main_encoder.py         # Main application for thick wire control (angle-based)
├── main_thin.py           # Main application for thin wire control (step-based)
├── config.py              # Centralized configuration for all parameters
├── hardware_manager.py    # Manages all motor communications (Stepper and Gripper)
├── sensor_manager.py      # Manages all sensor interactions (Visuotactile and Encoder)
├── utils.py               # Helper functions like angle conversion and data processing
└── README.md              # This documentation file
```

## Control Mode Details

### Thick Wire Control (2-5mm) - Angle-Based Mode

**Application**: `main_encoder.py`

**Control Strategy**:
- Uses calibrated **steps-per-degree** values for precise angular positioning
- Supports diameters: 2mm, 3mm, 4mm, 5mm
- Automatic control mode switching based on diameter selection
- High-precision encoder feedback for accurate positioning

**Key Features**:
- **Angular Target Control**: Set target angles (5° increments, up to 180°)
- **Calibrated Movement**: Uses regression-derived calibration data for each diameter
- **Directional Calibration**: Separate CW/CCW calibration values account for mechanical differences
- **Encoder Data Collection**: Real-time angular position monitoring
- **Automatic Displacement Calculation**: No manual encoder zeroing required

**Control Interface**:
```
CONTROLS (Thick Wire Mode):
  Rotation:  [A] CCW | [D] CW | [W/S] Speed | [Q/E] Target ±5°/±50
  Diameter:  [1] 1mm | [2/3/4/5] 2-5mm (switches to angle mode)
  Gripper:   [O] Open | [C] Close (Adaptive)
  Encoder:   [Z] Zero
  General:   [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit
```

**Calibration Data Example**:
```python
ENCODER_CALIBRATION = {
    'cw': {
        2: 0.652042,  # steps per degree for 2mm diameter
        3: 0.988245,  # steps per degree for 3mm diameter
        4: 1.276730,  # steps per degree for 4mm diameter
        5: 1.643381   # steps per degree for 5mm diameter
    },
    'ccw': {
        2: 0.653365,  # Separate calibration for CCW direction
        # ... similar structure for other diameters
    }
}
```

### Thin Wire Control (≤1mm) - Step-Based Mode

**Application**: `main_thin.py`

**Control Strategy**:
- Direct **step-based** control for maximum precision
- Supports ultra-thin wires: 0.2mm, 0.5mm, 1.0mm
- Enhanced with camera-based visual feedback
- Fine-grained control optimized for delicate manipulation

**Key Features**:
- **Step Target Control**: Set target steps (5-step increments)
- **Camera Integration**: Real-time visual feedback for wire positioning
- **Angle Stabilization**: Advanced filtering to reduce measurement noise
- **Delicate Handling**: Optimized for thin wire manipulation
- **Enhanced Recording**: Specialized data collection for thin wire analysis

**Control Interface**:
```
CONTROLS (Thin Wire Mode):
  Rotation:  [A] CCW | [D] CW | [W/S] Speed | [Q/E] Target ±5 steps
  Diameter:  [1] 0.2mm | [2] 0.5mm | [3] 1.0mm
  Gripper:   [O] Open | [C] Close (Adaptive)
  Recording: [P] Stop Recording (auto-stops after 5s)
  General:   [SPACE] STOP | [X] Zero Motor Pos | [ESC] Quit
```

**Visual Feedback System**:
- Real-time camera feed with angle detection
- Angle stabilization and noise filtering
- Visual overlay showing detected wire angle
- Automated wire centerline detection

## Mode Selection Logic

The system automatically determines the appropriate control mode based on wire diameter:

```python
def _determine_control_mode(diameter_mm):
    if diameter_mm <= 1.0:
        return "step"      # Thin wire mode - step-based control
    else:
        return "angle"     # Thick wire mode - angle-based control
```

**Thick Wire (2-5mm)**: 
- Run `python main_encoder.py`
- Angle-based control with encoder calibration
- Target angles in degrees with calibrated step conversion

**Thin Wire (≤1mm)**:
- Run `python main_thin.py` 
- Step-based control with visual feedback
- Direct step targeting for maximum precision

## Core Module Descriptions

### `config.py` - Centralized Configuration
- **Purpose**: Centralized configuration management for both control modes
- **Contents**:
  - Encoder calibration data (steps-per-degree for each diameter and direction)
  - Hardware configuration (COM ports, baud rates, motor settings)
  - Control mode parameters (angle increments, step increments, limits)
  - Adaptive gripping settings and sensor configuration
  - Recording and display settings

### `hardware_manager.py` - Motor Control
- **Purpose**: Manages all motor hardware communications
- **Components**:
  - `StepperMotorManager`: Dual-mode stepper control (angle-based and step-based)
    - `send_angle_move_command()`: For thick wire angle-based control
    - `send_step_move_command()`: For thin wire step-based control
  - `GripperMotorManager`: DM motor gripper control with adaptive gripping
  - `HardwareManager`: Coordinates both motor systems

### `sensor_manager.py` - Sensor Integration
- **Purpose**: Manages all sensor interactions and data collection
- **Components**:
  - `VisuotactileSensorManager`: Depth sensor operations and baseline calibration
  - `RotaryEncoderManager`: Enhanced encoder operations with automatic displacement calculation
    - No manual zeroing required
    - Automatic angle wrapping handling at 0°/360° boundary
    - Start-to-end displacement calculation for any encoder position
  - `SensorManager`: Coordinates all sensor systems

### `utils.py` - Helper Functions
- **Purpose**: Utility functions supporting both control modes
- **Key Functions**:
  - `get_steps_per_degree()`: Retrieves calibrated conversion factors
  - `calculate_angular_displacement()`: Advanced angle calculation with wrapping support
  - Angle conversion utilities and data processing functions
  - Mathematical calculations and file I/O operations

## Usage Instructions

### For Thick Wires (2-5mm) - Angle-Based Control

```bash
cd gripper_complete
python main_encoder.py
```

**Workflow**:
1. **Select Wire Diameter**: Press `[2]`, `[3]`, `[4]`, or `[5]` for 2-5mm diameters
2. **Set Target Angle**: Use `[Q/E]` to adjust target angle (±5° or ±50° increments)
3. **Position Wire**: Use gripper controls `[O]` open, `[C]` close with adaptive gripping
4. **Execute Rotation**: Press `[A]` for CCW or `[D]` for CW rotation
5. **Monitor Progress**: Real-time encoder feedback shows angular displacement
6. **Data Collection**: System automatically records encoder data for 5 seconds

**Key Features**:
- Automatic encoder displacement calculation (no manual zeroing required)
- Calibrated angular positioning using regression-derived step-per-degree values
- Real-time encoder feedback with angle wrapping support

### For Thin Wires (≤1mm) - Step-Based Control

```bash
cd gripper_complete
python main_thin.py
```

**Workflow**:
1. **Select Wire Diameter**: Press `[1]` for 0.2mm, `[2]` for 0.5mm, or `[3]` for 1.0mm
2. **Set Target Steps**: Use `[Q/E]` to adjust target steps (±5 step increments)
3. **Visual Setup**: Camera window shows real-time wire position with angle detection
4. **Position Wire**: Use gripper controls with enhanced sensitivity for thin wires
5. **Execute Movement**: Press `[A]` for CCW or `[D]` for CW step-based movement
6. **Visual Monitoring**: Watch camera feed for real-time wire angle and position

**Key Features**:
- Direct step control for maximum precision with delicate wires
- Real-time camera feedback with angle stabilization
- Specialized thin wire handling optimizations

### Common Controls (Both Modes)

- **Emergency Stop**: `[SPACE]` - Immediate motor stop
- **Speed Control**: `[W]` increase, `[S]` decrease motor speed  
- **Gripper**: `[O]` open, `[C]` close with adaptive pressure control
- **Sensor Calibration**: `[B]` calibrate depth sensor baseline
- **System Reset**: `[X]` zero motor position, `[ESC]` quit application

## Configuration

### Hardware Setup
Edit `config.py` to configure:
```python
CONFIG = {
    "stepper_port": "COM9",      # Arduino stepper controller
    "gripper_port": "COM3",      # DM motor gripper
    "encoder_port": "COM11",     # Rotary encoder
    "sensor_serial_id": 0,       # Depth sensor ID
    # ... other hardware settings
}
```

### Calibration Data
The system includes pre-calibrated step-per-degree values:
```python
ENCODER_CALIBRATION = {
    'cw': {2: 0.652042, 3: 0.988245, 4: 1.276730, 5: 1.643381},
    'ccw': {2: 0.653365, 3: 0.994351, 4: 1.292792, 5: 1.638945}
}
```

### Recording Settings
Configure data collection parameters:
```python
RECORDING_CONFIG = {
    "duration_seconds": 5.0,     # Auto-stop recording time
    "sampling_rate_hz": 20,      # Encoder reading frequency
}
```

## System Features & Improvements

### Advanced Encoder System
- **No Manual Zeroing Required**: Automatic start-to-end displacement calculation
- **Angle Wrapping Support**: Handles 0°/360° boundary crossings correctly
- **Dual-Direction Calibration**: Separate CW/CCW calibration accounts for mechanical differences
- **Real-Time Feedback**: Continuous encoder monitoring with stabilized readings

### Adaptive Control Systems
- **Diameter-Based Mode Selection**: Automatic switching between angle and step control
- **Calibrated Positioning**: Regression-derived calibration data for precise movements
- **Adaptive Gripping**: Pressure-sensitive gripper control with configurable thresholds
- **Visual Feedback**: Camera integration for thin wire applications

### Enhanced Data Collection
- **Automatic Recording**: 5-second encoder data collection with metadata
- **CSV Data Export**: Structured data output with operation details
- **Dashboard Display**: Real-time system status and operation feedback
- **Error Tracking**: Comprehensive error calculation and reporting

## Dependencies

**Core Requirements**:
- `pyserial` - Serial communication with Arduino and encoder
- `pynput` - Keyboard input handling for real-time control
- `minimalmodbus` - Modbus communication for encoder interface
- `numpy` - Numerical operations and data processing

**Optional Enhancements**:
- `opencv-python` - Camera integration for thin wire mode
- `matplotlib` - Data visualization and plotting capabilities
- `dmrobotics` - Depth sensor library (if available)
- `DM_CAN` - DM motor communication library
- `wmi` - Windows camera detection (thin wire mode only)

## Hardware Compatibility

### Supported Hardware
- **Stepper Motors**: Arduino-controlled dual stepper system
- **Gripper**: DM Motor with CAN communication
- **Encoder**: Modbus-compatible rotary encoder
- **Depth Sensor**: Compatible visuotactile sensors
- **Camera**: USB webcam for thin wire visual feedback

### Port Configuration
- Default COM ports configurable in `config.py`
- Automatic hardware detection and graceful degradation
- Connection status monitoring and error reporting

## Troubleshooting

### Common Issues

**Hardware Connection Problems**:
- Check COM port assignments in `config.py`
- Verify hardware power and cable connections
- Monitor connection status in system dashboard

**Calibration Issues**:
- Encoder calibration data pre-configured for tested diameters
- Manual recalibration possible through data collection
- Separate CW/CCW calibration handles directional differences

**Performance Optimization**:
- Adjust `sampling_rate_hz` for encoder reading frequency
- Configure `sensor_image_update_rate_hz` to balance visual feedback and CPU usage
- Enable/disable sensor images via `show_sensor_images` setting

### Debug Mode
Enable detailed logging:
```python
# In config.py
DISPLAY_CONFIG = {
    "update_interval_seconds": 0.1,  # Faster display updates
    "show_debug_info": True,         # Additional debug output
}
```
