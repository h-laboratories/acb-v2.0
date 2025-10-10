#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config_manager.h"
#include "MA730GQ.h"

class CommandManager {
public:
    // Constructor
    CommandManager(BLDCMotor* motor, MA730GQ* encoder);
    
    // Main command processing function
    void process_serial_commands();
    
    // Broadcast data handling
    
    // Command handlers
    void handle_set_position(float position);
    void handle_set_velocity(float velocity);
    void handle_set_torque(float torque);
    void handle_get_position();
    void handle_get_velocity();
    void handle_get_torque();
    void handle_enable();
    void handle_disable();
    void handle_home();
    void handle_stop();
    void handle_cmd_mode(int mode);
    void handle_get_velocity_pid();
    void handle_set_velocity_pid(float p, float i, float d);
    void handle_get_angle_pid();
    void handle_set_angle_pid(float p, float i, float d);
    void handle_get_current_pid();
    void handle_set_current_pid(float p, float i, float d);
    void handle_save_config();
    void handle_get_downsample();
    void handle_set_downsample(int downsample);
    void handle_get_temperature();
    void handle_get_bus_voltage();
    void handle_get_internal_temperature();
    void handle_recalibrate_sensors();
    void handle_drv8323_fault_check();
    void handle_get_pole_pairs();
    void handle_set_pole_pairs(int pole_pairs);
    void handle_reset_position();
    void handle_get_current_a();
    void handle_get_current_b();
    void handle_get_current_c();
    void handle_get_full_state();
    void handle_get_encoder_mag_status();
    void handle_reset_config_defaults();
    void handle_reset();
    void handle_help();
    void handle_get_version();
    void handle_get_min_angle();
    void handle_set_min_angle(float min_angle);
    void handle_get_max_angle();
    void handle_set_max_angle(float max_angle);
    
    // Utility functions
    float q88_to_float(uint16_t q88);
    uint16_t float_to_q88(float value);
    float q412_to_float(uint16_t q412);
    uint16_t float_to_q412(float value);
    
    // Command mode variables
    int command_mode;

private:
    BLDCMotor* motor_;
    MA730GQ* encoder_;
    // Private command parsing functions
    void parse_human_readable_command(String command);
    void parse_binary_command();
};

#endif // COMMAND_MANAGER_H
