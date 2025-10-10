#include "CommandManager.h"
#include "DRV8323RSRGZR.h"
#include <SimpleFOC.h>

CommandManager::CommandManager(BLDCMotor* motor, MA730GQ* encoder)
    : motor_(motor), encoder_(encoder), command_mode(1) {
}

void CommandManager::process_serial_commands() {
    if (Serial.available()) {
        if (command_mode == 1) {
            // Human readable mode
            String command = Serial.readStringUntil('\n');
            command.trim();
            parse_human_readable_command(command);
        } else if (command_mode == 2) {
            // Binary mode
            parse_binary_command();
        }
    }
}

void CommandManager::parse_human_readable_command(String command) {
    if (command.startsWith("set_position ")) {
        float pos = command.substring(12).toFloat();
        handle_set_position(pos);
    } else if (command.startsWith("set_velocity ")) {
        float vel = command.substring(12).toFloat();
        handle_set_velocity(vel);
    } else if (command.startsWith("set_torque ")) {
        float torque = command.substring(11).toFloat();
        handle_set_torque(torque);
    } else if (command == "get_position") {
        handle_get_position();
    } else if (command == "get_velocity") {
        handle_get_velocity();
    } else if (command == "get_torque") {
        handle_get_torque();
    } else if (command == "enable") {
        handle_enable();
    } else if (command == "disable") {
        handle_disable();
    } else if (command == "home") {
        handle_home();
    } else if (command == "stop") {
        handle_stop();
    } else if (command.startsWith("cmd_mode ")) {
        int mode = command.substring(9).toInt();
        handle_cmd_mode(mode);
    } else if (command == "get_full_state") {
        handle_get_full_state();
    } else if (command == "get_velocity_pid") {
        handle_get_velocity_pid();
    } else if (command.startsWith("set_velocity_pid ")) {
        String params = command.substring(16);
        params.trim();
        int firstSpace = params.indexOf(' ');
        int secondSpace = params.indexOf(' ', firstSpace + 1);
        if (firstSpace > 0 && secondSpace > 0) {
            float p = params.substring(0, firstSpace).toFloat();
            float i = params.substring(firstSpace + 1, secondSpace).toFloat();
            float d = params.substring(secondSpace + 1).toFloat();
            handle_set_velocity_pid(p, i, d);
        } else {
            Serial.print("set_velocity_pid: Invalid parameters. Received: ");
            Serial.println(params);
        }
    } else if (command == "get_angle_pid") {
        handle_get_angle_pid();
    } else if (command.startsWith("set_angle_pid ")) {
        String params = command.substring(14);
        params.trim();
        int firstSpace = params.indexOf(' ');
        int secondSpace = params.indexOf(' ', firstSpace + 1);
        if (firstSpace > 0 && secondSpace > 0) {
            float p = params.substring(0, firstSpace).toFloat();
            float i = params.substring(firstSpace + 1, secondSpace).toFloat();
            float d = params.substring(secondSpace + 1).toFloat();
            handle_set_angle_pid(p, i, d);
        } else {
            Serial.print("set_angle_pid: Invalid parameters. Received: ");
            Serial.println(params);
        }
    } else if (command == "get_current_pid") {
        handle_get_current_pid();
    } else if (command.startsWith("set_current_pid ")) {
        String params = command.substring(16);
        params.trim();
        int firstSpace = params.indexOf(' ');
        int secondSpace = params.indexOf(' ', firstSpace + 1);
        if (firstSpace > 0 && secondSpace > 0) {
            float p = params.substring(0, firstSpace).toFloat();
            float i = params.substring(firstSpace + 1, secondSpace).toFloat();
            float d = params.substring(secondSpace + 1).toFloat();
            handle_set_current_pid(p, i, d);
        } else {
            Serial.print("set_current_pid: Invalid parameters. Received: ");
            Serial.println(params);
        }
    } else if (command == "save_config") {
        handle_save_config();
    } else if (command == "get_downsample") {
        handle_get_downsample();
    } else if (command.startsWith("set_downsample ")) {
        int downsample = command.substring(15).toInt();
        handle_set_downsample(downsample);
    } else if (command == "get_temperature") {
        handle_get_temperature();
    } else if (command == "get_bus_voltage") {
        handle_get_bus_voltage();
    } else if (command == "get_internal_temperature") {
        handle_get_internal_temperature();
    } else if (command == "recalibrate_sensors") {
        handle_recalibrate_sensors();
    } else if (command == "drv8323_fault_check") {
        handle_drv8323_fault_check();
    } else if (command == "get_pole_pairs") {
        handle_get_pole_pairs();
    } else if (command.startsWith("set_pole_pairs ")) {
        int pole_pairs = command.substring(15).toInt();
        handle_set_pole_pairs(pole_pairs);
    } else if (command == "reset_position") {
        handle_reset_position();
    } else if (command == "get_current_a") {
        handle_get_current_a();
    } else if (command == "get_current_b") {
        handle_get_current_b();
    } else if (command == "get_current_c") {
        handle_get_current_c();
    } else if (command == "get_encoder_mag_status") {
        handle_get_encoder_mag_status();
    } else if (command == "reset_config_defaults") {
        handle_reset_config_defaults();
    } else if (command == "reset") {
        handle_reset();
    }
}

void CommandManager::parse_binary_command() {
    if (Serial.available() >= 3) {
        uint8_t cmd_id = Serial.read();
        uint8_t arg_low = Serial.read();
        uint8_t arg_high = Serial.read();
        uint16_t arg = (arg_high << 8) | arg_low;
        
        switch (cmd_id) {
            case 0x01: // set_position
                handle_set_position((float)arg);
                break;
            case 0x02: // set_velocity
                handle_set_velocity((float)arg);
                break;
            case 0x03: // set_torque
                handle_set_torque((float)arg);
                break;
            case 0x04: // get_position
                handle_get_position();
                break;
            case 0x05: // get_velocity
                handle_get_velocity();
                break;
            case 0x06: // get_torque
                handle_get_torque();
                break;
            case 0x07: // enable
                handle_enable();
                break;
            case 0x08: // disable
                handle_disable();
                break;
            case 0x09: // home
                handle_home();
                break;
            case 0x0A: // stop
                handle_stop();
                break;
            case 0x0B: // reset_position
                handle_reset_position();
                break;
            case 0x0C: // get_current_a
                handle_get_current_a();
                break;
            case 0x0D: // get_current_b
                handle_get_current_b();
                break;
            case 0x0E: // get_current_c
                handle_get_current_c();
                break;
            case 0xAB: // cmd_mode
                handle_cmd_mode((int)arg);
                break;
            case 0xAC: // reset_config_defaults
                handle_reset_config_defaults();
                break;
            case 0xAD: // reset
                handle_reset();
                break;
        }
    }
}


// Convert q8.8 fixed point to float
float CommandManager::q88_to_float(uint16_t q88) {
    return (float)((int16_t)q88) / 256.0f;
}

// Convert float to q8.8 fixed point
uint16_t CommandManager::float_to_q88(float value) {
    return (uint16_t)((int16_t)(value * 256.0f));
}

// Convert q4.12 fixed point to float
float CommandManager::q412_to_float(uint16_t q412) {
    return (float)((int16_t)q412) / 4096.0f;
}

// Convert float to q4.12 fixed point
uint16_t CommandManager::float_to_q412(float value) {
    return (uint16_t)((int16_t)(value * 4096.0f));
}

void CommandManager::handle_set_position(float position) {
    // Convert degrees to radians for motor control
    motor_->target = position * PI / 180.0f;
    motor_->controller = MotionControlType::angle;
    
    if (command_mode == 1) {
        Serial.print("set_position ");
        Serial.println(position);
    }
}

void CommandManager::handle_set_velocity(float velocity) {
    // Convert degrees/sec to radians/sec for motor control
    motor_->target = velocity * PI / 180.0f;
    motor_->controller = MotionControlType::velocity;
    
    if (command_mode == 1) {
        Serial.print("set_velocity ");
        Serial.println(motor_->target * 180.0f / PI);
    }
}

void CommandManager::handle_set_torque(float torque) {
    // Set torque target directly (in Amperes)
    motor_->target = torque;
    motor_->controller = MotionControlType::torque;
    
    if (command_mode == 1) {
        Serial.print("set_torque ");
        Serial.println(torque);
    }
}

void CommandManager::handle_get_position() {
    float current_position = motor_->shaft_angle * 180.0f / PI; // Convert to degrees
    if (command_mode == 1) {
        Serial.print("get_position ");
        Serial.println(current_position);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* pos_bytes = (uint8_t*)&current_position;
        for (int i = 0; i < 4; i++) {
            Serial.write(pos_bytes[i]);
        }
    }
}

void CommandManager::handle_get_velocity() {
    float current_velocity = motor_->shaft_velocity * 180.0f / PI; // Convert to degrees/sec
    if (command_mode == 1) {
        Serial.print("get_velocity ");
        Serial.println(current_velocity);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* vel_bytes = (uint8_t*)&current_velocity;
        for (int i = 0; i < 4; i++) {
            Serial.write(vel_bytes[i]);
        }
    }
}

void CommandManager::handle_get_torque() {
    float current_torque = motor_->shaft_velocity * 180.0f / PI; // Placeholder - would need actual torque measurement
    if (command_mode == 1) {
        Serial.print("get_torque ");
        Serial.println(current_torque);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* tor_bytes = (uint8_t*)&current_torque;
        for (int i = 0; i < 4; i++) {
            Serial.write(tor_bytes[i]);
        }
    }
}

void CommandManager::handle_enable() {
    motor_->enable();
    if (command_mode == 1) {
        Serial.println("enable");
    }
}

void CommandManager::handle_disable() {
    motor_->disable();
    if (command_mode == 1) {
        Serial.println("disable");
    }
}

void CommandManager::handle_home() {
    // Move to home position (0 degrees)
    motor_->target = 0.0f; // 0 radians
    motor_->controller = MotionControlType::angle;
    
    if (command_mode == 1) {
        Serial.println("home");
    }
}

void CommandManager::handle_stop() {
    // Hold current position
    motor_->target = motor_->shaft_angle; // Current position in radians
    motor_->controller = MotionControlType::angle;
    
    if (command_mode == 1) {
        Serial.println("stop");
    }
}

void CommandManager::handle_cmd_mode(int mode) {
    command_mode = mode;
    if (command_mode == 1) {
        Serial.println("cmd_mode 1");
    } else if (command_mode == 2) {
        Serial.println("cmd_mode 2");
    }
}


void CommandManager::handle_get_velocity_pid() {
    if (command_mode == 1) {
        Serial.print("get_velocity_pid ");
        Serial.print(acb_config.velocity_p);
        Serial.print(" ");
        Serial.print(acb_config.velocity_i);
        Serial.print(" ");
        Serial.println(acb_config.velocity_d);
    }
}

void CommandManager::handle_set_velocity_pid(float p, float i, float d) {
    acb_config.velocity_p = p;
    acb_config.velocity_i = i;
    acb_config.velocity_d = d;
    
    // Apply to motor controller
    motor_->PID_velocity.P = acb_config.velocity_p;
    motor_->PID_velocity.I = acb_config.velocity_i;
    motor_->PID_velocity.D = acb_config.velocity_d;
    
    if (command_mode == 1) {
        Serial.print("set_velocity_pid ");
        Serial.print(acb_config.velocity_p);
        Serial.print(" ");
        Serial.print(acb_config.velocity_i);
        Serial.print(" ");
        Serial.println(acb_config.velocity_d);
    }
}

void CommandManager::handle_get_angle_pid() {
    if (command_mode == 1) {
        Serial.print("get_angle_pid ");
        Serial.print(acb_config.angle_p);
        Serial.print(" ");
        Serial.print(acb_config.angle_i);
        Serial.print(" ");
        Serial.println(acb_config.angle_d);
    }
}

void CommandManager::handle_set_angle_pid(float p, float i, float d) {
    acb_config.angle_p = p;
    acb_config.angle_i = i;
    acb_config.angle_d = d;
    
    // Apply to motor controller
    motor_->P_angle.P = acb_config.angle_p;
    motor_->P_angle.I = acb_config.angle_i;
    motor_->P_angle.D = acb_config.angle_d;
    
    if (command_mode == 1) {
        Serial.print("set_angle_pid ");
        Serial.print(acb_config.angle_p);
        Serial.print(" ");
        Serial.print(acb_config.angle_i);
        Serial.print(" ");
        Serial.println(acb_config.angle_d);
    }
}

void CommandManager::handle_get_current_pid() {
    if (command_mode == 1) {
        Serial.print("get_current_pid ");
        Serial.print(acb_config.current_p);
        Serial.print(" ");
        Serial.print(acb_config.current_i);
        Serial.print(" ");
        Serial.println(acb_config.current_d);
    }
}

void CommandManager::handle_set_current_pid(float p, float i, float d) {
    acb_config.current_p = p;
    acb_config.current_i = i;
    acb_config.current_d = d;
    
    // Apply to motor controller
    motor_->PID_current_q.P = acb_config.current_p;
    motor_->PID_current_q.I = acb_config.current_i;
    motor_->PID_current_q.D = acb_config.current_d;
    
    if (command_mode == 1) {
        Serial.print("set_current_pid ");
        Serial.print(acb_config.current_p);
        Serial.print(" ");
        Serial.print(acb_config.current_i);
        Serial.print(" ");
        Serial.println(acb_config.current_d);
    }
}

void CommandManager::handle_save_config() {
    saveConfig();
    if (command_mode == 1) {
        Serial.println("save_config");
    }
}

void CommandManager::handle_get_downsample() {
    if (command_mode == 1) {
        Serial.print("get_downsample ");
        Serial.println(motor_->motion_downsample);
    }
}

void CommandManager::handle_set_downsample(int downsample) {
    motor_->motion_downsample = downsample;
    if (motor_->motion_downsample < 1) motor_->motion_downsample = 1; // Minimum value of 1
    if (command_mode == 1) {
        Serial.print("set_downsample ");
        Serial.println(motor_->motion_downsample);
    }
}

void CommandManager::handle_get_temperature() {
    extern float board_temperature;
    if (command_mode == 1) {
        Serial.print("get_temperature ");
        Serial.println(board_temperature);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* temp_bytes = (uint8_t*)&board_temperature;
        for (int i = 0; i < 4; i++) {
            Serial.write(temp_bytes[i]);
        }
    }
}

void CommandManager::handle_get_bus_voltage() {
    extern float bus_voltage;
    if (command_mode == 1) {
        Serial.print("get_bus_voltage ");
        Serial.println(bus_voltage);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* voltage_bytes = (uint8_t*)&bus_voltage;
        for (int i = 0; i < 4; i++) {
            Serial.write(voltage_bytes[i]);
        }
    }
}

void CommandManager::handle_get_internal_temperature() {
    extern float internal_temperature;
    if (command_mode == 1) {
        Serial.print("get_internal_temperature ");
        Serial.println(internal_temperature);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* temp_bytes = (uint8_t*)&internal_temperature;
        for (int i = 0; i < 4; i++) {
            Serial.write(temp_bytes[i]);
        }
    }
}

void CommandManager::handle_recalibrate_sensors() {
    if (command_mode == 1) {
        Serial.println("recalibrate_sensors: Starting sensor recalibration...");
    }
    SimpleFOCDebug::enable(&Serial);
    
    // Disable motor during calibration

    motor_->disable();
    delay(100);
    motor_->current_sense->skip_align = true;
    // Perform sensor alignment similar to initFOC
    // This will determine the zero electric angle and sensor direction
    motor_->voltage_sensor_align = 2;
    motor_->velocity_index_search = 5;
  
    motor_->sensor_direction = Direction::UNKNOWN;  // Start with default direction
    motor_->zero_electric_angle = NOT_SET;  // Reset to default
    
    // Enable motor for calibration
    motor_->enable();
    delay(1000);
    
    // Perform the calibration by calling initFOC
    // This will automatically determine the correct zero electric angle and sensor direction
    motor_->initFOC();
        
    // Store the calibration values in config (but don't save to EEPROM yet)
    acb_config.zero_electric_angle = motor_->zero_electric_angle;
    acb_config.sensor_direction = (motor_->sensor_direction == Direction::CW) ? 1 : -1;
    
    if (command_mode == 1) {
        Serial.print("recalibrate_sensors: Calibration complete. Zero electric angle: ");
        Serial.print(acb_config.zero_electric_angle, 4);
        Serial.print(", Sensor direction: ");
        Serial.println(acb_config.sensor_direction);
        
        // Print pole pairs and encoder rotation information
        Serial.print("recalibrate_sensors: Motor pole pairs: ");
        Serial.println(MOTOR_POLE_PAIRS);
        
        Serial.print("recalibrate_sensors: Encoder rotation direction: ");
        if (motor_->sensor_direction == Direction::CW) {
            Serial.println("Clockwise (CW)");
        } else if (motor_->sensor_direction == Direction::CCW) {
            Serial.println("Counter-Clockwise (CCW)");
        } else {
            Serial.println("Unknown");
        }
        
        Serial.println("recalibrate_sensors: Use 'save_config' command to save calibration to EEPROM.");
    }
    motor_->disable();
}

void CommandManager::handle_drv8323_fault_check() {
    if (command_mode == 1) {
        Serial.println("drv8323_fault_check: Checking DRV8323 fault status...");
    }
    
    // Get DRV8323 instance from main firmware
    extern DRV8323RSRGZR drv8323;
    
    // Check for faults using the DRV8323 manager
    bool has_faults = drv8323.checkFaults();
    
    if (command_mode == 1) {
        if (has_faults) {
            Serial.println("drv8323_fault_check: Faults detected - see details above");
        } else {
            Serial.println("drv8323_fault_check: No faults detected");
        }
    }
}

void CommandManager::handle_get_pole_pairs() {
    if (command_mode == 1) {
        Serial.print("get_pole_pairs ");
        Serial.println(acb_config.pole_pairs);
    }
}

void CommandManager::handle_set_pole_pairs(int pole_pairs) {
    // Validate pole pairs value
    if (pole_pairs < 1 || pole_pairs > 50) {
        if (command_mode == 1) {
            Serial.print("set_pole_pairs: Invalid pole pairs value. Must be between 1 and 50. Received: ");
            Serial.println(pole_pairs);
        }
        return;
    }
    
    acb_config.pole_pairs = pole_pairs;
    
    // Update motor pole pairs
    motor_->pole_pairs = acb_config.pole_pairs;
    
    if (command_mode == 1) {
        Serial.print("set_pole_pairs ");
        Serial.println(acb_config.pole_pairs);
    }
}

void CommandManager::handle_reset_position() {
    // Reset the motor's shaft angle to 0 without changing the target
    // This effectively sets the current position as the new zero reference
    // motor_->shaft_angle = 0.0f;
    motor_->sensor_offset = motor_->sensor->getAngle();
    
    
    if (command_mode == 1) {
        Serial.println("reset_position");
    }
}

void CommandManager::handle_get_current_a() {
    extern float current_a;
    if (command_mode == 1) {
        Serial.print("get_current_a ");
        Serial.println(current_a);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* current_bytes = (uint8_t*)&current_a;
        for (int i = 0; i < 4; i++) {
            Serial.write(current_bytes[i]);
        }
    }
}

void CommandManager::handle_get_current_b() {
    extern float current_b;
    if (command_mode == 1) {
        Serial.print("get_current_b ");
        Serial.println(current_b);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* current_bytes = (uint8_t*)&current_b;
        for (int i = 0; i < 4; i++) {
            Serial.write(current_bytes[i]);
        }
    }
}

void CommandManager::handle_get_current_c() {
    extern float current_c;
    if (command_mode == 1) {
        Serial.print("get_current_c ");
        Serial.println(current_c);
    } else if (command_mode == 2) {
        // Send as raw float (4 bytes)
        uint8_t* current_bytes = (uint8_t*)&current_c;
        for (int i = 0; i < 4; i++) {
            Serial.write(current_bytes[i]);
        }
    }
}

void CommandManager::handle_get_full_state() {
    // Get current values
    float current_position = motor_->shaft_angle * 180.0f / PI; // Convert to degrees
    float current_velocity = motor_->shaft_velocity * 180.0f / PI; // Convert to degrees/sec
    float current_torque = motor_->shaft_velocity * 180.0f / PI; // Placeholder - would need actual torque measurement
    
    // Get status values (extern variables from main firmware)
    extern float board_temperature;
    extern float bus_voltage;
    extern float internal_temperature;
    extern float current_a;
    extern float current_b;
    extern float current_c;
    
    if (command_mode == 1) {
        // Human readable full state with all status information
        Serial.print("full_state ");
        Serial.print(current_position);
        Serial.print(" ");
        Serial.print(current_velocity);
        Serial.print(" ");
        Serial.print(current_torque);
        Serial.print(" ");
        Serial.print(board_temperature);
        Serial.print(" ");
        Serial.print(bus_voltage);
        Serial.print(" ");
        Serial.print(internal_temperature);
        Serial.print(" ");
        Serial.print(current_a);
        Serial.print(" ");
        Serial.print(current_b);
        Serial.print(" ");
        Serial.println(current_c);
    } else if (command_mode == 2) {
        // Binary full state - send as raw floats (4 bytes each)
        Serial.write(0xAE); // Full state data command ID
        uint8_t* pos_bytes = (uint8_t*)&current_position;
        uint8_t* vel_bytes = (uint8_t*)&current_velocity;
        uint8_t* tor_bytes = (uint8_t*)&current_torque;
        uint8_t* temp_bytes = (uint8_t*)&board_temperature;
        uint8_t* voltage_bytes = (uint8_t*)&bus_voltage;
        uint8_t* int_temp_bytes = (uint8_t*)&internal_temperature;
        uint8_t* current_a_bytes = (uint8_t*)&current_a;
        uint8_t* current_b_bytes = (uint8_t*)&current_b;
        uint8_t* current_c_bytes = (uint8_t*)&current_c;
        
        for (int i = 0; i < 4; i++) {
            Serial.write(pos_bytes[i]);
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(vel_bytes[i]);
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(tor_bytes[i]);
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(temp_bytes[i]);
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(voltage_bytes[i]);
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(int_temp_bytes[i]);
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(current_a_bytes[i]);
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(current_b_bytes[i]);
        }
        for (int i = 0; i < 4; i++) {
            Serial.write(current_c_bytes[i]);
        }
    }
}

void CommandManager::handle_get_encoder_mag_status() {    
    if (command_mode == 1) {
        uint8_t mag_status = encoder_->readRegister(27);

        Serial.print("get_encoder_mag_status ");
        Serial.print((mag_status >> 7) & 0x01);
        Serial.print(" ");
        Serial.print((mag_status >> 6) & 0x01);
        Serial.print(" ");
        Serial.print(!(((mag_status >> 2) & 0x01)) | ((((mag_status >> 3) & 0x01))));
        Serial.println();
    }
}

void CommandManager::handle_reset_config_defaults() {
    if (command_mode == 1) {
        Serial.println("reset_config_defaults: Resetting configuration to defaults...");
    }
    
    // Reset configuration to defaults
    resetConfig();
    
    // Apply the reset configuration to the motor
    motor_->PID_velocity.P = acb_config.velocity_p;
    motor_->PID_velocity.I = acb_config.velocity_i;
    motor_->PID_velocity.D = acb_config.velocity_d;
    
    motor_->P_angle.P = acb_config.angle_p;
    motor_->P_angle.I = acb_config.angle_i;
    motor_->P_angle.D = acb_config.angle_d;
    
    motor_->PID_current_q.P = acb_config.current_p;
    motor_->PID_current_q.I = acb_config.current_i;
    motor_->PID_current_q.D = acb_config.current_d;
    
    motor_->PID_current_d.P = acb_config.current_p;
    motor_->PID_current_d.I = acb_config.current_i;
    motor_->PID_current_d.D = acb_config.current_d;
    
    // Apply sensor calibration values
    motor_->zero_electric_angle = acb_config.zero_electric_angle;
    motor_->sensor_direction = (acb_config.sensor_direction == 1) ? Direction::CW : Direction::CCW;
    
    // Apply motor configuration
    motor_->pole_pairs = acb_config.pole_pairs;
    
    if (command_mode == 1) {
        Serial.println("reset_config_defaults: Configuration reset to defaults and applied to motor");
    }
}

void CommandManager::handle_reset() {
    if (command_mode == 1) {
        Serial.println("reset: Performing complete STM32G4 system reset...");
    }
    
    // Give a brief moment for the message to be sent
    delay(100);
    
    // Perform complete system reset using NVIC_SystemReset()
    NVIC_SystemReset();
}