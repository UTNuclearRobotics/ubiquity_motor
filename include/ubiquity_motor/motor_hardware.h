/**
Copyright (c) 2016, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef MOTORHARDWARE_H
#define MOTORHARDWARE_H

#include "hardware_interface/system_interface.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "ubiquity_motor/msg/motor_state.hpp"

#include <diagnostic_updater/update_functions.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <motor_parameters.hpp>
#include <ubiquity_motor/motor_serial.h>

#include <gtest/gtest_prod.h>

// Mininum hardware versions required for various features
#define MIN_HW_OPTION_SWITCH 50

// The gear ratio defaults for wheels shipped with Magni
#define WHEEL_GEAR_RATIO_1  ((double)(4.294))   // Default original motor gear ratio for Magni
#define WHEEL_GEAR_RATIO_2  ((double)(5.170))   // 2nd version standard Magni wheels gear ratio

// For experimental purposes users will see that the wheel encoders are three phases
// of very neaar 43 pulses per revolution or about 43*3 edges so we see very about 129 ticks per rev
// This leads to 129/(2*Pi)  or about 20.53 ticks per radian experimentally.
// 60 ticks per revolution of the motor (pre gearbox) and a gear ratio of 4.2941176:1 for 1st rev Magni wheels
// An unexplained constant from the past I leave here till explained is: 17.2328767

#define TICKS_PER_RAD_FROM_GEAR_RATIO   ((double)(4.774556)*(double)(2.0))  // Used to generate ticks per radian 
#define TICKS_PER_RADIAN_DEFAULT        41.004  // For runtime use  getWheelGearRatio() * TICKS_PER_RAD_FROM_GEAR_RATIO    
#define WHEEL_GEAR_RATIO_DEFAULT        WHEEL_GEAR_RATIO_1

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

class MotorHardware : public hardware_interface::SystemInterface {
public:
using CommsParams = ubiquity_motor::Params::CommsParams;
using FirmwareParams = ubiquity_motor::Params::FirmwareParams;

    MotorHardware();
    
    // System Interface interface
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override; 
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override; 
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // ROS interface
    void initMcbParameters();

    // Motor hardware interface
    void closePort();
    bool openPort();
    void clearCommands();
    void readInputs();
    void writeSpeeds();
    void writeSpeedsInRadians(double left_radians, double right_radians);
    void publishFirmwareInfo();
    float calculateBatteryPercentage(float voltage, int cells, const float* type);
    bool areWheelSpeedsLower(double wheelSpeedRadPerSec);
    void requestFirmwareVersion();
    void requestFirmwareDate();
    void setParams(ubiquity_motor::Params::FirmwareParams firmware_params);
    void sendParams();
    void forcePidParamUpdates();
    float getBatteryVoltage(void);
    void setDeadmanTimer(int32_t deadman);
    void setDeadzoneEnable(int32_t deadzone_enable);
    void setDebugLeds(bool led1, bool led2);
    void setHardwareVersion(int32_t hardware_version);
    void setEstopPidThreshold(int32_t estop_pid_threshold);
    void setEstopDetection(int32_t estop_detection);
    bool getEstopState(void);
    void setMaxFwdSpeed(int32_t max_speed_fwd);
    void setMaxRevSpeed(int32_t max_speed_rev);
    void setMaxPwm(int32_t max_pwm);
    void setWheelType(int32_t wheel_type);
    void setWheelGearRatio(double wheel_gear_ratio);
    double getWheelGearRatio(void);
    double getWheelTicksPerRadian(void);
    void setDriveType(int32_t drive_type);
    void setPidControl(int32_t pid_control);
    void nullWheelErrors(void);
    void setWheelDirection(int32_t wheel_direction);
    void getMotorCurrents(double &currentLeft, double &currentRight);
    int  getOptionSwitch(void);
    int  getPidControlWord(void);
    void setOptionSwitchReg(int32_t option_switch);
    void requestSystemEvents();
    void setSystemEvents(int32_t system_events);
    void getWheelJointPositions(double &leftWheelPosition, double &rightWheelPosition);
    void setWheelJointVelocities(double leftWheelVelocity, double rightWheelVelocity);
    void publishMotorState(void);
    int firmware_version{0};
    int firmware_date{0};
    int firmware_options{0};
    int num_fw_params;  // This is used for sendParams as modulo count
    int hardware_version;
    int estop_pid_threshold;
    int max_speed_fwd;
    int max_speed_rev;
    int max_pwm;
    int pid_control;
    int deadman_enable;
    int system_events;
    int wheel_type;
    double wheel_gear_ratio{WHEEL_GEAR_RATIO_DEFAULT};
    int drive_type;
    int wheel_slip_events{0};
    double left_last_wheel_pos {0.0};
    double right_last_wheel_pos{0.0};
    bool wheel_slip_nulling{false};

    bool last_mcb_enabled = false;
    rclcpp::Time last_joint_time{0};
    rclcpp::Time last_sys_maint_time{0};
    rclcpp::Duration estop_release_delay{0, 0};
    const rclcpp::Duration estop_release_dead_time{std::chrono::milliseconds(800)};

private:
    void _addOdometryRequest(std::vector<MotorMessage>& commands) const;
    void _addVelocityRequest(std::vector<MotorMessage>& commands) const;

    int16_t calculateSpeedFromRadians(double radians);
    double calculateRadiansFromTicks(int16_t ticks);

    std::vector<hardware_interface::StateInterface> joint_state_interface_;
    std::vector<hardware_interface::CommandInterface> velocity_joint_interface_;

    rclcpp::Node::SharedPtr nh_;

    ubiquity_motor::ParamListener params_listener_;
    ubiquity_motor::Params all_params_;
    ubiquity_motor::Params::FirmwareParams prev_fw_params;
    ubiquity_motor::Params::FirmwareParams& fw_params = all_params_.firmware_params;
    ubiquity_motor::Params::NodeParams& node_params = all_params_.node_params;
    ubiquity_motor::Params::CommsParams& serial_params = all_params_.comms_params;

    // Control loop delay
    std::chrono::milliseconds mcb_status_period_{20};
    std::chrono::milliseconds control_loop_period_{100};
    rclcpp::Duration max_cycle_time = 1.25*control_loop_period_;
    rclcpp::Duration min_cycle_time = 0.75*control_loop_period_;

    // A period where if wheels are under stress for this long we back off stress
    rclcpp::Duration wheel_slip_nulling_period_{std::chrono::seconds(2)};  

    // Record of how long the wheels go at zero velocity
    rclcpp::Duration zero_velocity_time_{rclcpp::Duration(0, 0)};

    public: diagnostic_updater::Updater diag_updater;
    private:

    int32_t deadman_timer;

    double  ticks_per_radian{TICKS_PER_RADIAN_DEFAULT}; // Odom ticks per radian for wheel encoders in use

    int32_t sendPid_count;

    bool estop_motor_power_off{false};    // Motor power inactive, most likely from ESTOP switch

    struct Joint {
        double position;
        double velocity;
        double effort;
        double velocity_command;

        Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
    };
    std::array<Joint, 2> joints_;

    // MessageTypes enum for refering to motor or wheel number
    enum MotorOrWheelNumber {
        Motor_M1 = 1,
        Motor_M2 = 2
    };

    // MessageTypes enum in class to avoid global namespace pollution
    enum WheelJointLocation {
        Left  = 0,
        Right = 1
    };

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_error_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_current_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_tick_interval_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_tick_interval_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr firmware_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_power_active_pub_;
    rclcpp::Publisher<ubiquity_motor::msg::MotorState>::SharedPtr motor_state_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_control_sub_;
    void SystemControlCallback(std_msgs::msg::String::ConstSharedPtr msg);

    rclcpp::TimerBase::SharedPtr param_update_timer_;

    std::unique_ptr<MotorSerial> motor_serial_;

    MotorDiagnostics motor_diag_;

    FRIEND_TEST(MotorHardwareTests, nonZeroWriteSpeedsOutputs);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPosition);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPositionMax);
};

#endif
