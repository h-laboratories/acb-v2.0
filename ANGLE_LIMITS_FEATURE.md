# ACB v2.0 Angle Limits Feature

## Overview
This feature adds configurable minimum and maximum angle limits to the ACB v2.0 motor controller firmware. The system prevents the motor from moving outside the specified angle range and automatically switches to position mode when limits are exceeded during velocity or torque control.

## Features

### 1. Configurable Angle Limits
- **Minimum Angle**: Configurable minimum allowed position (default: -180°)
- **Maximum Angle**: Configurable maximum allowed position (default: 180°)
- **Persistent Storage**: Limits are saved to EEPROM and restored on boot
- **Validation**: Prevents setting invalid limits (min ≥ max)

### 2. Position Command Validation
- **Range Checking**: `set_position` commands are validated against current limits
- **Error Messages**: Clear error messages when attempting to set positions outside limits
- **No Movement**: Motor position is not changed if the command is outside limits

### 3. Automatic Limit Enforcement
- **Velocity Mode**: When in velocity mode, if the motor position exceeds limits, it automatically switches to position mode at the nearest limit
- **Torque Mode**: Same behavior as velocity mode
- **Serial Notification**: Sends `set_position X` message when limits are enforced
- **Real-time Monitoring**: Continuously monitors position during velocity/torque control

## New Commands

### Get/Set Minimum Angle
```
get_min_angle                    # Get current minimum angle limit
set_min_angle <degrees>          # Set minimum angle limit
```

### Get/Set Maximum Angle
```
get_max_angle                    # Get current maximum angle limit
set_max_angle <degrees>          # Set maximum angle limit
```

## Configuration

### Default Values
- **Minimum Angle**: -180°
- **Maximum Angle**: 180°

### EEPROM Storage
- Angle limits are stored in the `ACBConfig` structure
- Automatically saved when using `save_config` command
- Restored from EEPROM on system boot

## Usage Examples

### Setting Up Angle Limits
```
# Set limits for a 90-degree range
set_min_angle -45
set_max_angle 45
save_config

# Verify limits
get_min_angle
get_max_angle
```

### Testing Position Commands
```
# These will work (within limits)
set_position 0
set_position 30
set_position -30

# These will fail with error messages
set_position 50    # Outside max limit
set_position -50   # Outside min limit
```

### Velocity Mode with Limits
```
# Set velocity mode
set_velocity 10
enable

# If motor moves outside [-45, 45], it will automatically:
# 1. Switch to position mode
# 2. Hold at the nearest limit (-45 or 45)
# 3. Send "set_position X" message via serial
```

## Implementation Details

### Files Modified
1. **config_manager.h**: Added `min_angle` and `max_angle` to `ACBConfig` structure
2. **config_manager.cpp**: Added initialization and printing of angle limits
3. **config.h**: Added default angle limit constants
4. **CommandManager.h**: Added new command handler declarations
5. **CommandManager.cpp**: Implemented angle limit commands and validation
6. **acb_v2.0_firmware.ino**: Added angle limit monitoring in main loop

### Key Functions
- `handle_get_min_angle()`: Returns current minimum angle limit
- `handle_set_min_angle()`: Sets minimum angle limit with validation
- `handle_get_max_angle()`: Returns current maximum angle limit
- `handle_set_max_angle()`: Sets maximum angle limit with validation
- `check_angle_limits()`: Monitors position and enforces limits
- Enhanced `handle_set_position()`: Validates position against limits

### Safety Features
- **Input Validation**: Prevents setting invalid angle ranges
- **Real-time Monitoring**: Continuous position checking during motion
- **Automatic Switching**: Seamless transition to position mode when limits exceeded
- **Clear Feedback**: Serial messages indicate when limits are enforced

## Testing

A test script `test_angle_limits.py` is provided to verify all functionality:
- Setting and getting angle limits
- Position command validation
- Velocity mode limit enforcement
- Configuration persistence

## Binary Protocol Support

The angle limit commands are also supported in binary mode:
- `0xB0`: get_min_angle
- `0xB1`: set_min_angle
- `0xB2`: get_max_angle
- `0xB3`: set_max_angle

## Error Handling

### Invalid Limit Setting
```
set_min_angle: Error - min_angle (100) must be less than max_angle (90)
set_max_angle: Error - max_angle (-100) must be greater than min_angle (-90)
```

### Position Outside Limits
```
set_position: Error - position (150) is outside allowed range [-90, 90]
```

## Integration Notes

- Angle limits are applied to all position commands
- Limits are enforced in real-time during velocity and torque control
- Configuration is persistent across power cycles
- Compatible with existing firmware features
- No impact on performance when limits are not exceeded
