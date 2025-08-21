# Gripper Control System - Modular Architecture

This is a refactored, modular version of the original `gripper_angle_control_encoder.py` script. The system has been broken down into logical modules for better maintainability, testability, and extensibility.

## Architecture Overview

```
gripper_complete/
├── main.py                 # Main application entry point, handles UI and the main loop
├── config.py               # Centralized configuration for all parameters
├── hardware_manager.py     # Manages all motor communications (Stepper and Gripper)
├── sensor_manager.py       # Manages all sensor interactions (Visuotactile and Encoder)
├── utils.py                # Helper functions like angle conversion and data processing
└── README.md               # This documentation file
```

## Module Descriptions

### `main.py`
- **Purpose**: Main application entry point and coordination
- **Responsibilities**:
  - System initialization and cleanup
  - UI display and keyboard input handling
  - Main control loop coordination
  - Thread management
  - Signal handling for graceful shutdown

### `config.py`
- **Purpose**: Centralized configuration management
- **Contents**:
  - All system parameters and constants
  - Encoder calibration data
  - Hardware configuration (ports, baud rates, etc.)
  - Adaptive gripping settings
  - Recording configuration
  - Display settings
  - File paths

### `hardware_manager.py`
- **Purpose**: Manages all motor hardware communications
- **Components**:
  - `StepperMotorManager`: Handles stepper motor control via Arduino
  - `GripperMotorManager`: Handles DM motor gripper control
  - `HardwareManager`: Coordinates both motor systems

### `sensor_manager.py`
- **Purpose**: Manages all sensor interactions
- **Components**:
  - `VisuotactileSensorManager`: Handles depth sensor operations
  - `RotaryEncoderManager`: Handles rotary encoder operations
  - `SensorManager`: Coordinates all sensor systems

### `utils.py`
- **Purpose**: Helper functions and utilities
- **Functions**:
  - Angle conversion utilities
  - Data processing and plotting
  - File I/O operations
  - Mathematical calculations

## Key Improvements

### 1. **Modularity**
- Clear separation of concerns
- Each module has a single responsibility
- Easy to test individual components
- Simple to extend with new features

### 2. **Configuration Management**
- All parameters centralized in `config.py`
- Easy to modify settings without touching code
- Environment-specific configurations possible

### 3. **Error Handling**
- Graceful degradation when hardware is unavailable
- Proper cleanup on shutdown
- Better error reporting and recovery

### 4. **Thread Safety**
- Proper thread management
- Clean shutdown procedures
- Reduced race conditions

### 5. **Maintainability**
- Clear class hierarchies
- Well-documented functions
- Consistent coding style
- Reduced code duplication

## Usage

### Running the System

```bash
cd gripper_complete
python main.py
```

### Configuration

Edit `config.py` to modify:
- Hardware ports and settings
- Calibration parameters
- Recording settings
- Display preferences

### Adding New Features

1. **New Hardware**: Add to `hardware_manager.py`
2. **New Sensors**: Add to `sensor_manager.py`
3. **New Utilities**: Add to `utils.py`
4. **New Configuration**: Add to `config.py`

## Dependencies

The system requires the same dependencies as the original script:

- `pyserial` - Serial communication
- `pynput` - Keyboard input handling
- `minimalmodbus` - Modbus communication
- `opencv-python` - Image processing
- `numpy` - Numerical operations
- `matplotlib` - Plotting
- `dmrobotics` - Sensor library (if available)
- `DM_CAN` - DM motor library (if available)

## Migration from Original Script

The refactored system maintains full compatibility with the original functionality:

- Same keyboard controls
- Same hardware interfaces
- Same data recording capabilities
- Same adaptive gripping behavior

### Key Differences

1. **Import Structure**: Uses relative imports within the module
2. **State Management**: Centralized in `SystemState` class
3. **Error Handling**: More robust with proper cleanup
4. **Configuration**: Externalized to `config.py`

## Testing

Each module can be tested independently:

```python
# Test hardware manager
from hardware_manager import HardwareManager
hw = HardwareManager()
status = hw.initialize()

# Test sensor manager
from sensor_manager import SensorManager
sensors = SensorManager()
status = sensors.initialize()

# Test utilities
from utils import angle_to_steps
steps = angle_to_steps(90, 3, 'cw')
```

## Troubleshooting

### Common Issues

1. **Import Errors**: Ensure all dependencies are installed
2. **Hardware Not Found**: Check port configurations in `config.py`
3. **Permission Errors**: Run with appropriate permissions for hardware access
4. **Thread Issues**: Check for proper cleanup in error conditions

### Debug Mode

Add debug logging by modifying the configuration:

```python
# In config.py
DEBUG_MODE = True
```

## Future Enhancements

The modular architecture enables easy addition of:

1. **GUI Interface**: Replace console UI with graphical interface
2. **Network Communication**: Add remote control capabilities
3. **Data Logging**: Enhanced logging and analytics
4. **Plugin System**: Modular hardware/sensor support
5. **Configuration UI**: Graphical configuration management

## Contributing

When adding new features:

1. Follow the existing module structure
2. Add appropriate error handling
3. Update configuration as needed
4. Document new functionality
5. Test thoroughly before committing

## License

This refactored system maintains the same license as the original script.
