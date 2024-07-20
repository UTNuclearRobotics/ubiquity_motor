#include "ubiquity_motor/motor_diagnostics.hpp"
#include "ubiquity_motor/motor_message.h"

// Diagnostics Status Updater Functions
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_msgs::msg::DiagnosticStatus;

void MotorDiagnostics::battery_status(DiagnosticStatusWrapper &stat) {
    stat.add("Battery Voltage", battery_voltage);
    if (battery_voltage < battery_voltage_low_level) {
        stat.summary(DiagnosticStatusWrapper::WARN, "Battery low");
    }
    else if (battery_voltage < battery_voltage_critical) {
        stat.summary(DiagnosticStatusWrapper::ERROR, "Battery critical");
    }
    else {
        stat.summary(DiagnosticStatusWrapper::OK, "Battery OK");
    }
}

// PID parameters for motor control
void MotorDiagnostics::motor_pid_p_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam P", fw_pid_proportional);
    stat.summary(DiagnosticStatus::OK, "PID Parameter P");
}
void MotorDiagnostics::motor_pid_i_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam I", fw_pid_integral);
    stat.summary(DiagnosticStatus::OK, "PID Parameter I");
}
void MotorDiagnostics::motor_pid_d_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam D", fw_pid_derivative);
    stat.summary(DiagnosticStatus::OK, "PID Parameter D");
}
void MotorDiagnostics::motor_pid_v_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam V", fw_pid_velocity);
    stat.summary(DiagnosticStatus::OK, "PID Parameter V");
}
void MotorDiagnostics::motor_max_pwm_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam MaxPWM", fw_max_pwm);
    stat.summary(DiagnosticStatus::OK, "PID Max PWM");
}


void MotorDiagnostics::motor_power_status(DiagnosticStatusWrapper &stat) {
    stat.add("Motor Power", !estop_motor_power_off);
    if (estop_motor_power_off == false) {
        stat.summary(DiagnosticStatusWrapper::OK, "Motor power on");
    }
    else {
        stat.summary(DiagnosticStatusWrapper::WARN, "Motor power off");
    }
}


// Show firmware options and give readable decoding of the meaning of the bits
void MotorDiagnostics::firmware_options_status(DiagnosticStatusWrapper &stat) {
    stat.add("Firmware Options", firmware_options);
    std::string option_descriptions("");
    if (firmware_options & MotorMessage::OPT_ENC_6_STATE) {
        option_descriptions += "High resolution encoders";
    } else {
        option_descriptions += "Standard resolution encoders";
    }
    if (firmware_options & MotorMessage::OPT_WHEEL_TYPE_THIN) {
        option_descriptions +=  ", Thin gearless wheels";
    } else {
        option_descriptions +=  ", Standard wheels";
    }
    if (firmware_options & MotorMessage::OPT_DRIVE_TYPE_4WD) {
        option_descriptions +=  ", 4 wheel drive";
    } else {
        option_descriptions +=  ", 2 wheel drive";
    }
    if (firmware_options & MotorMessage::OPT_WHEEL_DIR_REVERSE) {
        // Only indicate wheel reversal if that has been set as it is non-standard
        option_descriptions +=  ", Reverse polarity wheels";
    }
    stat.summary(DiagnosticStatusWrapper::OK, option_descriptions);
}