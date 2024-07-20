#ifndef MOTORDIAGNOSTICS_HPP
#define MOTORDIAGNOSTICS_HPP

#include <diagnostic_updater/update_functions.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

struct MotorDiagnostics {
    MotorDiagnostics()
        : odom_update_status(
              diagnostic_updater::FrequencyStatusParam(&odom_min_freq, &odom_max_freq)) {}
    // Communication Statuses
    int firmware_version = 0;
    int firmware_date    = 0;
    int firmware_options = 0;

    // These are for diagnostic topic output 
    int fw_pid_proportional = 0;
    int fw_pid_integral = 0;
    int fw_pid_derivative = 0;
    int fw_pid_control = 0;
    int fw_pid_velocity = 0;
    int fw_pid_denominator = 0;
    int fw_pid_moving_buffer_size = 0;
    int fw_max_pwm = 0;
   
    double odom_max_freq = 1000;
    double odom_min_freq = 50;
    diagnostic_updater::FrequencyStatus odom_update_status;

    // Limits
    bool left_pwm_limit = false;
    bool right_pwm_limit = false;
    bool left_integral_limit = false;
    bool right_integral_limit = false;
    bool left_max_speed_limit = false;
    bool right_max_speed_limit = false;
    bool param_limit_in_firmware = false;

    // Power supply statuses
    float battery_voltage = 0.0;
    float battery_voltage_low_level = 22.5;
    float battery_voltage_critical = 21.0;

    // Wheel current states
    double motorCurrentLeft  = 0.0;
    double motorCurrentRight = 0.0;

    // ADC count for zero current. We could calibrate this if required. 
    // Nominally 1024 and goes up from there this lower value is used. 
    double motorAmpsZeroAdcCount = 1015;    

    int    motorPwmDriveLeft  = 0;
    int    motorPwmDriveRight = 0;

    /* For later implementation (firmware support)
    bool  main_5V_error = false;
    bool  main_5V_ol = false;
    bool  main_12V_error = false;
    bool  main_12V_ol = false;
    bool  aux_5V_error = false;
    bool  aux_5V_ol = false;
    bool  aux_12V_error = false;
    bool  aux_12V_ol = false;
    */

    bool  estop_motor_power_off = false;  // for Diagnostic reporting of ESTOP switch

    void firmware_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void limit_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void battery_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_power_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_p_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_i_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_d_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_v_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_max_pwm_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void firmware_options_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void firmware_date_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

#endif
