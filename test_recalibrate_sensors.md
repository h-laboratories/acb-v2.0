# Test Plan for recalibrate_sensors Command

## Overview
This document outlines the test plan for the new `recalibrate_sensors` command that has been added to the ACB v2.0 firmware.

## Implementation Summary
The `recalibrate_sensors` command has been implemented with the following features:

1. **Configuration Structure Updates**:
   - Added `zero_electric_angle` (float) to store the calibrated zero electric angle
   - Added `sensor_direction` (int) to store the sensor direction (1 for CW, -1 for CCW)

2. **Command Implementation**:
   - Added `handle_recalibrate_sensors()` function to CommandManager
   - Added command parsing for "recalibrate_sensors" in human-readable mode
   - Updated help text to include the new command

3. **Calibration Process**:
   - Disables motor during calibration
   - Resets sensor direction and zero electric angle to defaults
   - Calls `motor.initFOC()` to perform automatic calibration
   - Stores the resulting calibration values in config (but not EEPROM)
   - Provides feedback to user about calibration results

4. **EEPROM Storage**:
   - Calibration values are stored in the config structure
   - Values are only saved to EEPROM when user sends `save_config` command
   - On startup, stored calibration values are applied to the motor

## Test Commands

### Basic Functionality Test
1. Connect to the ACB v2.0 via serial terminal
2. Send command: `recalibrate_sensors`
3. Verify output:
   ```
   recalibrate_sensors: Starting sensor recalibration...
   recalibrate_sensors: Calibration complete. Zero electric angle: [value], Sensor direction: [value]
   recalibrate_sensors: Use 'save_config' command to save calibration to EEPROM.
   ```

### Save Configuration Test
1. After running `recalibrate_sensors`
2. Send command: `save_config`
3. Verify output: `save_config`
4. Power cycle the device
5. Verify that the calibration values are preserved

### Configuration Display Test
1. Send command: `printConfig` (if available) or check startup messages
2. Verify that sensor calibration values are displayed:
   ```
   Sensor Calibration: Zero Electric Angle=[value], Sensor Direction=[value]
   ```

## Expected Behavior

### During Calibration
- Motor should be disabled during the calibration process
- The `initFOC()` function should determine the correct zero electric angle and sensor direction
- Calibration values should be stored in the config structure but not immediately saved to EEPROM

### After Calibration
- Motor should be ready to use with the new calibration values
- User should be prompted to save configuration to EEPROM
- Calibration values should persist across power cycles only after `save_config` is called

### Error Handling
- If calibration fails, appropriate error messages should be displayed
- Motor should be safely disabled if calibration fails

## Integration with Existing Commands

The `recalibrate_sensors` command integrates with:
- `save_config`: Saves calibration values to EEPROM
- Existing PID and motion control commands
- Motor enable/disable functionality

## Notes
- The calibration process is similar to what happens during `initFOC()` in the setup
- Calibration values are stored as floats for zero electric angle and integers for sensor direction
- The command only works in human-readable mode (command_mode = 1)
- Binary mode support could be added in the future if needed
