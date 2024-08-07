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
#include <ubiquity_motor/motor_hardware.h>
#include <ubiquity_motor/motor_message.h>
#include <ubiquity_motor/motor_parameters.h>
#include <boost/assign.hpp>
#include <boost/math/special_functions/round.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// To access I2C we need some system includes
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#define  I2C_DEVICE  "/dev/i2c-1"     // This is specific to default Magni I2C port on host
const static uint8_t  I2C_PCF8574_8BIT_ADDR = 0x40; // I2C addresses are 7 bits but often shown as 8-bit

//#define SENSOR_DISTANCE 0.002478

// TODO: Make HIGH_SPEED_RADIANS, WHEEL_VELOCITY_NEAR_ZERO and ODOM_4WD_ROTATION_SCALE  all ROS params
#define HIGH_SPEED_RADIANS        (1.8)               // threshold to consider wheel turning 'very fast'
#define WHEEL_VELOCITY_NEAR_ZERO  ((double)(0.08))
#define ODOM_4WD_ROTATION_SCALE   ((double)(1.65))    // Used to correct for 4WD skid rotation error

#define MOTOR_AMPS_PER_ADC_COUNT   ((double)(0.0238)) // 0.1V/Amp  2.44V=1024 count so 41.97 cnt/amp

#define VELOCITY_READ_PER_SECOND   ((double)(10.0))   // read = ticks / (100 ms), so we scale of 10 for ticks/second
#define LOWEST_FIRMWARE_VERSION 28

namespace ubiquity_motor {

// Debug verification use only
int32_t  g_odomLeft  = 0;
int32_t  g_odomRight = 0;
int32_t  g_odomEvent = 0;

//lead acid battery percentage levels for a single cell
const static float SLA_AGM[11] = {
    1.800, // 0
    1.837, // 10
    1.875, // 20
    1.912, // 30
    1.950, // 40
    2.010, // 50
    2.025, // 60
    2.062, // 70
    2.100, // 80
    2.137, // 90
    2.175, // 100
};


//li-ion battery percentage levels for a single cell
const static float LI_ION[11] = {
    3.3, // 0
    3.49, // 10
    3.53, // 20
    3.55, // 30
    3.60, // 40
    3.64, // 50
    3.70, // 60
    3.80, // 70
    3.85, // 80
    4.05, // 90
    4.20, // 100
};

// We sometimes need to know if we are rotating in place due to special ways of dealing with
// A 4wd robot must skid to turn. This factor approximates the actual rotation done vs what
// the wheel encoders have indicated.  This only applies if in 4WD mode
double   g_odom4wdRotationScale = ODOM_4WD_ROTATION_SCALE;

// 4WD robot chassis that has to use extensive torque to rotate in place and due to wheel slip has odom scale factor
double   g_radiansLeft  = 0.0;
double   g_radiansRight = 0.0;

// This utility opens and reads 1 or more bytes from a device on an I2C bus
// This method was taken on it's own from a big I2C class we may choose to use later
static rclcpp::Logger static_logger = rclcpp::get_logger("MotorHardware");
static int i2c_BufferRead(const char *i2cDevFile, uint8_t i2c8bitAddr,
                          uint8_t *pBuffer, int16_t chipRegAddr, uint16_t NumBytesToRead);

MotorHardware::MotorHardware() :
    nh_(std::make_shared<rclcpp::Node>("magni_motor_hardware")),
    params_listener_(nh_),
    mcb_interface_(fw_params),
    diag_updater(nh_)
{
    // Configure logger for static methods
    static_logger = nh_->get_logger();

    // Load params from ROS parameter server
    all_params_ = params_listener_.get_params();

    rclcpp::QoS latching_qos(1);
    latching_qos.transient_local();
    left_error_pub_ = nh_->create_publisher<std_msgs::msg::Int32>("left_error", 1);
    right_error_pub_ = nh_->create_publisher<std_msgs::msg::Int32>("right_error", 1);
    left_tick_interval_pub_  = nh_->create_publisher<std_msgs::msg::Int32>("left_tick_interval", 1);
    right_tick_interval_pub_ = nh_->create_publisher<std_msgs::msg::Int32>("right_tick_interval", 1);
    left_current_pub_ = nh_->create_publisher<std_msgs::msg::Float32>("left_current", 1);
    right_current_pub_ = nh_->create_publisher<std_msgs::msg::Float32>("right_current", 1);
    firmware_state_pub_ = nh_->create_publisher<std_msgs::msg::String>("firmware_version", latching_qos);
    battery_state_pub_ = nh_->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 1);
    motor_power_active_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("motor_power_active", 1);
    motor_state_pub_ = nh_->create_publisher<ubiquity_motor::msg::MotorState>("motor_state", 1);

    system_control_sub_ = nh_->create_subscription<std_msgs::msg::String>(ROS_TOPIC_SYSTEM_CONTROL, 1000, 
        std::bind(&MotorHardware::SystemControlCallback, this, std::placeholders::_1));

    // Update parameters every 500 milliseconds
    param_update_timer_ = rclcpp::create_timer(nh_, nh_->get_clock(), std::chrono::milliseconds(500),
        [this] () -> void {
            if (params_listener_.is_old(all_params_)) {
                all_params_ = params_listener_.get_params();
            }
        }
    );
    
    // Configure diagnostics
    MotorDiagnostics* diag = mcb_interface_.getBaseDiagnosticInterface();
    diag_updater.setHardwareID("Motor Controller");
    diag_updater.add("Firmware"       , diag, &MotorDiagnostics::firmware_status);
    diag_updater.add("Limits"         , diag, &MotorDiagnostics::limit_status);
    diag_updater.add("Battery"        , diag, &MotorDiagnostics::battery_status);
    diag_updater.add("MotorPower"     , diag, &MotorDiagnostics::motor_power_status);
    diag_updater.add("PidParamP"      , diag, &MotorDiagnostics::motor_pid_p_status);
    diag_updater.add("PidParamI"      , diag, &MotorDiagnostics::motor_pid_i_status);
    diag_updater.add("PidParamD"      , diag, &MotorDiagnostics::motor_pid_d_status);
    diag_updater.add("PidParamV"      , diag, &MotorDiagnostics::motor_pid_v_status);
    diag_updater.add("PidMaxPWM"      , diag, &MotorDiagnostics::motor_max_pwm_status);
    diag_updater.add("FirmwareOptions", diag, &MotorDiagnostics::firmware_options_status);
    diag_updater.add("FirmwareDate"   , diag, &MotorDiagnostics::firmware_date_status);
}

auto MotorHardware::on_init(const hardware_interface::HardwareInfo& info) -> CallbackReturn {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Magni has exactly three states and one command interface on each joint
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                nh_->get_logger(),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                nh_->get_logger(),
                "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(
                nh_->get_logger(),
                "Joint '%s' has %zu state interfaces. 3 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                nh_->get_logger(),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                nh_->get_logger(),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(
                nh_->get_logger(),
                "Joint '%s' have '%s' as third state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
}

auto MotorHardware::on_configure(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) -> CallbackReturn 
{
    // Initialize communication with the MCB
    RCLCPP_INFO(nh_->get_logger(), "Delay before MCB serial port initialization");
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(nh_->get_logger(), "Initialize MCB serial port '%s' for %ld baud",
        serial_params.serial_port.c_str(), serial_params.baud_rate);

    while (rclcpp::ok() && !motor_serial_){
        try{
            motor_serial_ = std::make_unique<MotorSerial>(serial_params.serial_port, serial_params.baud_rate);
            break;
        } catch (const serial::IOException& e) {
            RCLCPP_FATAL_THROTTLE(nh_->get_logger(), *(nh_->get_clock()), 1000,
                "Error opening serial port %s, trying again", serial_params.serial_port.c_str());
        }
    }
    RCLCPP_INFO(nh_->get_logger(), "MCB serial port initialized");

    // Retrieve data from the MCB
    mcb_interface_.requestFirmwareVersion();
    rclcpp::sleep_for(mcb_status_period_);
    mcb_interface_.readInputs();

    diag_updater.broadcast(0, "Establishing communication with motors");
    // Start times counter at 1 to prevent false error print (0 % n = 0)
    for (int times = 1; rclcpp::ok() && 0 == mcb_interface_.firmware_version; times++) {
        if (times % 30 == 0) {
            RCLCPP_ERROR(nh_->get_logger(), "The Firmware not reporting its version");
        }
        mcb_interface_.requestFirmwareVersion();
        rclcpp::sleep_for(mcb_status_period_);
        mcb_interface_.readInputs();
    }

    if (mcb_interface_.firmware_version >= MIN_FW_FIRMWARE_DATE) {
        // If supported by firmware also request date code for this version
        RCLCPP_INFO(nh_->get_logger(), "Requesting Firmware daycode ");
        mcb_interface_.requestFirmwareDate();
    }

    RCLCPP_INFO(nh_->get_logger(), "Initializing MCB parameters...");
    initMcbParameters();
    RCLCPP_INFO(nh_->get_logger(), "Initialization of MCB completed.");

    RCLCPP_INFO(nh_->get_logger(), "Clearing system events counters");
    if (mcb_interface_.firmware_version >= MIN_FW_SYSTEM_EVENTS) {
        // Start out with zero for system events
        mcb_interface_.setSystemEvents(0);  // Clear entire system events register
        mcb_interface_.system_events = 0;
        rclcpp::sleep_for(mcb_status_period_);
    }

    // Send out the refreshable firmware parameters, most are the PID terms
    // We must be sure num_fw_params is set to the modulo used in sendParams()
    RCLCPP_INFO(nh_->get_logger(), "Sending params to MCB");
    mcb_interface_.sendParams();

    // Clear any commands the robot has at this time
    clearCommands();

    // Record the initial state of the wheels
    auto [left_wheel_pos, right_wheel_pos] = mcb_interface_.getWheelJointPositions();
    left_last_wheel_pos = left_wheel_pos;
    right_last_wheel_pos = right_wheel_pos;

    RCLCPP_INFO(nh_->get_logger(), "Starting motor control node now");

    return CallbackReturn::SUCCESS;
}

auto MotorHardware::on_cleanup(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) -> CallbackReturn 
{
    RCLCPP_INFO(nh_->get_logger(), "Disconnecting MCB serial port and destroying MotorSerial object");
    motor_serial_.reset();
    return CallbackReturn::SUCCESS;
}

auto MotorHardware::on_activate(
    [[maybe_unused]] const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
{
    return CallbackReturn::SUCCESS;
}

auto MotorHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state) -> CallbackReturn
{
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t idx = 0; idx < info_.joints.size(); idx++) {
        state_interfaces.emplace_back(
            info_.joints[idx].name, hardware_interface::HW_IF_POSITION, &mcb_interface_.joints[idx].position
        );
        state_interfaces.emplace_back(
            info_.joints[idx].name, hardware_interface::HW_IF_VELOCITY, &mcb_interface_.joints[idx].velocity
        );
        state_interfaces.emplace_back(
            info_.joints[idx].name, hardware_interface::HW_IF_EFFORT, &mcb_interface_.joints[idx].effort
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t idx = 0; idx < info_.joints.size(); idx++) {
        command_interfaces.emplace_back(
            info_.joints[idx].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_[idx]
        );
    }

    return command_interfaces;
}

hardware_interface::return_type MotorHardware::read(
    [[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& elapsed_time)
{
    // Determine and set wheel velocities in rad/sec from hardware positions in rads
    const rclcpp::Duration joint_update_period = std::chrono::milliseconds(250);
    auto joint_elapsed_time = time - last_joint_time;
    if (joint_elapsed_time > joint_update_period) {
        last_joint_time = nh_->now();
        auto [left_wheel_pos, right_wheel_pos] = mcb_interface_.getWheelJointPositions();
        double leftWheelVel  = (left_wheel_pos  - left_last_wheel_pos)  / joint_elapsed_time.seconds();
        double rightWheelVel = (right_wheel_pos - right_last_wheel_pos) / joint_elapsed_time.seconds();
        mcb_interface_.setWheelJointVelocities(leftWheelVel, rightWheelVel); // rad/sec
        left_last_wheel_pos  = left_wheel_pos;
        right_last_wheel_pos = right_wheel_pos;

        // Publish motor state at this time
        publishMotorState();

        // Implement static wheel slippage relief
        // Deal with auto-null of MCB wheel setpoints if wheel slip nulling is enabled
        // We null wheel torque if wheel speed has been very low for a long time
        // This would be even better if we only did this when over a certain current is heating the wheel
        if (mcb_interface_.wheel_slip_nulling) {
            if (areWheelSpeedsLower(node_params.wheel_slip_threshold)) {
                zero_velocity_time_ = zero_velocity_time_ + joint_update_period;   // add to time at zero velocity
                if (zero_velocity_time_ > wheel_slip_nulling_period_) {
                    // null wheel error if at zero velocity for the nulling check period
                    // OPTION: We could also just null wheels at high wheel power
                    RCLCPP_DEBUG(nh_->get_logger(), "Applying wheel slip relief now with slip period of %4.1f sec ",
                        wheel_slip_nulling_period_.seconds());
                    mcb_interface_.wheel_slip_events += 1;
                    mcb_interface_.nullWheelErrors();
                    zero_velocity_time_ = rclcpp::Duration(0, 0);   // reset time we have been at zero velocity
                }
            } else {
                zero_velocity_time_ = rclcpp::Duration(0, 0);   // reset time we have been at zero velocity
            }
        }
    }

    // Read in and process all serial packets that came from the MCB.
    // The MotorSerial class has been receiving and queing packets from the MCB
    mcb_interface_.readInputs();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorHardware::write(
    const rclcpp::Time& time, const rclcpp::Duration& elapsed_loop_time)
{
    // Special handling if motor control is disabled.  skip the entire loop
    if (!node_params.mcb_control_enabled) {
        // Check for if we are just starting to go into idle mode and release MCB
        if (last_mcb_enabled) {
            RCLCPP_WARN(nh_->get_logger(), "Motor controller going offline and closing MCB serial port");
            mcb_interface_.closePort();
        }
        last_mcb_enabled = false;
        return hardware_interface::return_type::OK;
    }

    // Were disabled but re-enabled so re-setup mcb
    if (!last_mcb_enabled) {        
        last_mcb_enabled = true;
        RCLCPP_WARN(nh_->get_logger(), "Motor controller went from offline to online!");

        // Must re-open serial port
        if (mcb_interface_.openPort()) {
            mcb_interface_.setSystemEvents(0);  // Clear entire system events register
            mcb_interface_.system_events = 0;
            rclcpp::sleep_for(mcb_status_period_);
            
            // Setup MCB parameters that are defined by host parameters in most cases
            initMcbParameters();
            RCLCPP_WARN(nh_->get_logger(), "Motor controller has been re-initialized as we go back online");
        } else {
            // We do not have recovery from this situation and it seems not possible
            RCLCPP_ERROR(nh_->get_logger(), "ERROR in re-opening of the Motor controller!");
        }
    }

    if (elapsed_loop_time < min_cycle_time || elapsed_loop_time > max_cycle_time) {
        RCLCPP_WARN(nh_->get_logger(), "Resetting controller due to time jump %f seconds",
                     elapsed_loop_time.seconds());
        clearCommands();
    }
    mcb_interface_.sendParams();

    // Periodically watch for MCB board having been reset which is an MCB system event
    // This is also a good place to refresh or show status that may have changed
    const rclcpp::Duration maint_elapsed = time - last_sys_maint_time;
    const rclcpp::Duration sys_maint_period = std::chrono::seconds(60);
    if (maint_elapsed > sys_maint_period) {
        mcb_interface_.requestSystemEvents();
        rclcpp::sleep_for(mcb_status_period_);
        last_sys_maint_time = nh_->now();

        // See if we are in a low battery voltage state
        const std::string batStatus = (mcb_interface_.getBatteryVoltage() < fw_params.battery_voltage.low_level) ? "LOW!" : "OK";

        // Post a status message for MCB state periodically. This may be nice to do more on as required
        RCLCPP_INFO(nh_->get_logger(), "Battery = %5.2f volts [%s], MCB system events 0x%x,  PidCtrl 0x%x, WheelType '%s' DriveType '%s' GearRatio %6.3f",
            mcb_interface_.getBatteryVoltage(), batStatus.c_str(), mcb_interface_.system_events, mcb_interface_.getPidControlWord(),
            (mcb_interface_.getWheelType() == MotorMessage::OPT_WHEEL_TYPE_THIN) ? "thin" : "standard",
            node_params.drive_type.c_str(), mcb_interface_.getWheelGearRatio());

        // If we detect a power-on of MCB we should re-initialize MCB
        if ((mcb_interface_.system_events & MotorMessage::SYS_EVENT_POWERON) != 0) {
            RCLCPP_WARN(nh_->get_logger(), "Detected Motor controller PowerOn event!");
            mcb_interface_.setSystemEvents(0);  // Clear entire system events register
            mcb_interface_.system_events = 0;
            rclcpp::sleep_for(mcb_status_period_);
            
            // Setup MCB parameters that are defined by host parameters in most cases
            initMcbParameters();
            RCLCPP_WARN(nh_->get_logger(), "Motor controller has been re-initialized");
        }

        // A periodic refresh of wheel type which is a safety net due to it's importance.
        // This can be removed when a solid message protocol is developed
        if (mcb_interface_.firmware_version >= MIN_FW_WHEEL_TYPE_THIN) {
            // Refresh the wheel type setting
            mcb_interface_.setWheelType(mcb_interface_.getWheelType());
            rclcpp::sleep_for(mcb_status_period_);
        }
        // a periodic refresh of drive type which is a safety net due to it's importance.
        if (mcb_interface_.firmware_version >= MIN_FW_DRIVE_TYPE) {
            // Refresh the drive type setting
            mcb_interface_.setDriveType(mcb_interface_.getDriveType());
            rclcpp::sleep_for(mcb_status_period_);
        }
    }

    // Update motor controller speeds unless global disable is set, perhaps for colision safety
    if (mcb_interface_.getEstopState()) {
        mcb_interface_.writeSpeedsInRadians(0.0, 0.0);    // We send zero velocity when estop is active
        estop_release_delay = estop_release_dead_time;
    } else {
        if (estop_release_delay.seconds() > 0.0) {
            // Implement a delay after estop release where velocity remains zero
            estop_release_delay = estop_release_delay - elapsed_loop_time;
            mcb_interface_.writeSpeedsInRadians(0.0, 0.0);
        } else {
            if (node_params.mcb_speed_enabled) {        // A global disable used for safety at node level
                mcb_interface_.writeSpeedsInRadians(    // Normal operation using current system speeds
                    joint_velocity_commands_[0], 
                    joint_velocity_commands_[1]
                );
            } else {
                mcb_interface_.writeSpeedsInRadians(0.0, 0.0);
            }
        }
    }

    diag_updater.force_update();
    return hardware_interface::return_type::OK;
}

void MotorHardware::SystemControlCallback(std_msgs::msg::String::ConstSharedPtr msg) {
    RCLCPP_DEBUG(nh_->get_logger(), "System control msg with content: '%s']", msg->data.c_str());

    // Manage the complete cut-off of all control to the MCB 
    // Typically used for firmware upgrade, but could be used for other diagnostics
    if (msg->data.find(MOTOR_CONTROL_CMD) != std::string::npos) {
        if (msg->data.find(MOTOR_CONTROL_ENABLE) != std::string::npos) {;
            if (node_params.mcb_control_enabled == 0) {  // Only show message if state changes
                RCLCPP_INFO(nh_->get_logger(), "Received System control msg to ENABLE control of the MCB");
            }
            node_params.mcb_control_enabled = 1;
        } else if (msg->data.find(MOTOR_CONTROL_DISABLE) != std::string::npos) {
            if (node_params.mcb_control_enabled != 0) {  // Only show message if state changes
                RCLCPP_INFO(nh_->get_logger(), "Received System control msg to DISABLE control of the MCB");
            }
            node_params.mcb_control_enabled = 0;
        }
    }

    // Manage a motor speed override used to allow collision detect motor stopping
    if (msg->data.find(MOTOR_SPEED_CONTROL_CMD) != std::string::npos) {
        if (msg->data.find(MOTOR_CONTROL_ENABLE) != std::string::npos) {;
            if (node_params.mcb_speed_enabled == 0) {  // Only show message if state changes
                RCLCPP_INFO(nh_->get_logger(), "Received System control msg to ENABLE control of motor speed");
            }
            node_params.mcb_speed_enabled = 1;
        } else if (msg->data.find(MOTOR_CONTROL_DISABLE) != std::string::npos) {
            if (node_params.mcb_speed_enabled != 0) {  // Only show message if state changes
                RCLCPP_INFO(nh_->get_logger(), "Received System control msg to DISABLE control of motor speed");
            }
            node_params.mcb_speed_enabled = 0;
        }
     }
}

//  initMcbParameters()
//
//  Setup MCB parameters that are from host level options or settings
//
void MotorHardware::initMcbParameters()
{
    // A full mcb initialization requires high level system overrides to be disabled
    node_params.mcb_control_enabled = 1;
    node_params.mcb_speed_enabled   = 1;

    // Force future calls to sendParams() to update current pid parametes on the MCB
    mcb_interface_.forcePidParamUpdates();

    // Determine the wheel type to be used by the robot base 
    MotorMessage::HwOptions wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
    if (node_params.wheel_type == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(nh_->get_logger(), "Default wheel_type of 'standard' will be used.");
        wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
    } else {
        // Any other setting leads to host setting the wheel type
        if (node_params.wheel_type == "standard") {
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
            RCLCPP_INFO(nh_->get_logger(), "Host is specifying wheel_type of '%s'", "standard");
        } else if (node_params.wheel_type == "thin"){
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_THIN;
            RCLCPP_INFO(nh_->get_logger(), "Host is specifying wheel_type of '%s'", "thin");

            // If thin wheels and no drive_type is in yaml file we will use 4wd
            if (node_params.drive_type == "firmware_default") {
                RCLCPP_INFO(nh_->get_logger(), "Default to drive_type of 4wd when THIN wheels unless option drive_type is set");
                node_params.drive_type = "4wd";
            }
        } else {
            RCLCPP_WARN(nh_->get_logger(), "Invalid wheel_type of '%s' specified! Using wheel type of standard", 
                node_params.wheel_type.c_str());
            node_params.wheel_type = "standard";
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
        }
    }
    // Write out the wheel type setting to hardware layer
    mcb_interface_.setWheelType(wheel_type);
    rclcpp::sleep_for(mcb_status_period_);

    // Determine the wheel gear ratio to be used by the robot base 
    // Firmware does not use this setting so no message to firmware is required
    // This gear ratio is contained in the hardware layer so if this node got new setting update hardware layer
    mcb_interface_.setWheelGearRatio(node_params.wheel_gear_ratio);
    RCLCPP_INFO(nh_->get_logger(), "Wheel gear ratio of %5.3f will be used.", node_params.wheel_gear_ratio);

    // Determine the drive type to be used by the robot base
    int32_t drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
    if (node_params.drive_type == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(nh_->get_logger(), "Default drive_type of 'standard' will be used.");
        drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
    } else {
        // Any other setting leads to host setting the drive type
        if (node_params.drive_type == "standard") {
            drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
            RCLCPP_INFO(nh_->get_logger(), "Host is specifying drive_type of '%s'", "standard");
        } else if (node_params.drive_type == "4wd"){
            drive_type = MotorMessage::OPT_DRIVE_TYPE_4WD;
            RCLCPP_INFO(nh_->get_logger(), "Host is specifying drive_type of '%s'", "4wd");
        } else {
            RCLCPP_WARN(nh_->get_logger(), "Invalid drive_type of '%s' specified! Using drive type of standard",
                node_params.drive_type.c_str());
            node_params.drive_type = "standard";
            drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
        }
    }
    // Write out the drive type setting to hardware layer
    mcb_interface_.setDriveType(drive_type);
    drive_type = drive_type;
    rclcpp::sleep_for(mcb_status_period_);

    int32_t wheel_direction = 0;
    if (node_params.wheel_direction == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(nh_->get_logger(), "Firmware default wheel_direction will be used.");
    } else {
        // Any other setting leads to host setting the wheel type
        if (node_params.wheel_direction == "standard") {
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
            RCLCPP_INFO(nh_->get_logger(), "Host is specifying wheel_direction of '%s'", "standard");
        } else if (node_params.wheel_direction == "reverse"){
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_REVERSE;
            RCLCPP_INFO(nh_->get_logger(), "Host is specifying wheel_direction of '%s'", "reverse");
        } else {
            RCLCPP_WARN(nh_->get_logger(), "Invalid wheel_direction of '%s' specified! Using wheel direction of standard", 
                node_params.wheel_direction.c_str());
            node_params.wheel_direction = "standard";
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
        }
        // Write out the wheel direction setting
        mcb_interface_.setWheelDirection(wheel_direction);
        rclcpp::sleep_for(mcb_status_period_);
    }

    // Tell the controller board firmware what version the hardware is at this time.
    // TODO: Read from I2C.   At this time we only allow setting the version from ros parameters
    if (mcb_interface_.firmware_version >= MIN_FW_HW_VERSION_SET) {
        RCLCPP_INFO_ONCE(nh_->get_logger(), "Firmware is version %d. Setting Controller board version to %ld", 
            mcb_interface_.firmware_version, fw_params.controller_board_version);
        mcb_interface_.setHardwareVersion(fw_params.controller_board_version);
        RCLCPP_DEBUG(nh_->get_logger(), "Controller board version has been set to %ld", 
            fw_params.controller_board_version);
        rclcpp::sleep_for(mcb_status_period_);
    }

    // Suggest to customer to have current firmware version
    if (mcb_interface_.firmware_version < MIN_FW_SUGGESTED) {
        RCLCPP_ERROR_ONCE(nh_->get_logger(), "Firmware is version V%d. We strongly recommend minimum firmware version of at least V%d", 
            mcb_interface_.firmware_version, MIN_FW_SUGGESTED);
        rclcpp::sleep_for(mcb_status_period_);
    } else {
        RCLCPP_INFO_ONCE(nh_->get_logger(), "Firmware is version V%d. This meets the recommend minimum firmware versionof V%d", 
            mcb_interface_.firmware_version, MIN_FW_SUGGESTED);
        rclcpp::sleep_for(mcb_status_period_);
    }

    // Certain 4WD robots rely on wheels to skid to reach final positions.
    // For such robots when loaded down the wheels can get in a state where they cannot skid.
    // This leads to motor overheating.  This code below sacrifices accurate odometry which
    // is not achievable in such robots anyway to relieve high wattage drive power when zero velocity.
    mcb_interface_.wheel_slip_nulling = false;
    if ((mcb_interface_.firmware_version >= MIN_FW_WHEEL_NULL_ERROR) && (node_params.drive_type == "4wd")) {
        mcb_interface_.wheel_slip_nulling = true;
        RCLCPP_INFO(nh_->get_logger(), "Wheel slip nulling will be enabled for this 4wd system when velocity remains at zero.");
    }

    // Tell the MCB board what the port that is on the Pi I2c says on it (the mcb cannot read it's own switchs!)
    // We could re-read periodically but perhaps only every 5-10 sec but should do it from main loop
    if (mcb_interface_.firmware_version >= MIN_FW_OPTION_SWITCH && mcb_interface_.hardware_version >= MIN_HW_OPTION_SWITCH) {
        fw_params.option_switch = mcb_interface_.getOptionSwitch();
        RCLCPP_INFO(nh_->get_logger(), "Setting firmware option register to 0x%lx.", fw_params.option_switch);
        mcb_interface_.setOptionSwitchReg(fw_params.option_switch);
        rclcpp::sleep_for(mcb_status_period_);
    }
    
    if (mcb_interface_.firmware_version >= MIN_FW_SYSTEM_EVENTS) {
        // Start out with zero for system events
        mcb_interface_.setSystemEvents(0);  // Clear entire system events register
        mcb_interface_.system_events = 0;
        rclcpp::sleep_for(mcb_status_period_);
    }

    // Setup other firmware parameters that could come from ROS parameters
    if (mcb_interface_.firmware_version >= MIN_FW_ESTOP_SUPPORT) {
        mcb_interface_.setEstopPidThreshold(fw_params.estop_pid_threshold);
        rclcpp::sleep_for(mcb_status_period_);
        mcb_interface_.setEstopDetection(fw_params.estop_detection);
        rclcpp::sleep_for(mcb_status_period_);
    }

    if (mcb_interface_.firmware_version >= MIN_FW_MAX_SPEED_AND_PWM) {
        mcb_interface_.setMaxFwdSpeed(fw_params.max_speed_fwd);
        rclcpp::sleep_for(mcb_status_period_);
        mcb_interface_.setMaxRevSpeed(fw_params.max_speed_rev);
        rclcpp::sleep_for(mcb_status_period_);
    }
        
    return;
}

void MotorHardware::clearCommands() {
    std::fill(joint_velocity_commands_.begin(), joint_velocity_commands_.end(), 0.0);
}

// Publish motor state conditions
void MotorHardware::publishMotorState(void) {
    ubiquity_motor::msg::MotorState mstateMsg;

    mstateMsg.header.frame_id = "";   // Could be base_link.  We will use empty till required
    mstateMsg.header.stamp    = nh_->now();

    const auto [left_current, right_current] = mcb_interface_.getMotorCurrents();
    const auto [left_drive  , right_drive  ] = mcb_interface_.getWheelPWMDrives();

    mstateMsg.left_position     = mcb_interface_.joints[WheelJointLocation::Left].position;
    mstateMsg.right_position    = mcb_interface_.joints[WheelJointLocation::Right].position;
    mstateMsg.left_rotate_rate  = mcb_interface_.joints[WheelJointLocation::Left].velocity;
    mstateMsg.right_rotate_rate = mcb_interface_.joints[WheelJointLocation::Right].velocity;
    mstateMsg.left_current      = left_current;
    mstateMsg.right_current     = right_current;
    mstateMsg.left_pwm_drive    = left_drive;
    mstateMsg.right_pwm_drive   = right_drive;
    motor_state_pub_->publish(mstateMsg);
    return;
}

void MotorHardware::publishFirmwareInfo(){
    //publish the firmware version and optional date to ROS
    if(mcb_interface_.firmware_version > 0){

        std_msgs::msg::String fstate;
        fstate.data = "v"+std::to_string(mcb_interface_.firmware_version);

        if(mcb_interface_.firmware_date > 0){
            std::stringstream stream;
            stream << std::hex << mcb_interface_.firmware_date;
            std::string daycode(stream.str());

            fstate.data +=" "+daycode;
        }

        firmware_state_pub_->publish(fstate);
    }
}

// areWheelSpeedsLower()  Determine if all wheel joint speeds are below given threshold
bool MotorHardware::areWheelSpeedsLower(double wheelSpeedRadPerSec) {
    // This call pulls in speeds from the joints array maintained by other layers

    const double left_radians  = joint_velocity_commands_[WheelJointLocation::Left];
    const double right_radians = joint_velocity_commands_[WheelJointLocation::Right];

    return ((std::abs(left_radians) < wheelSpeedRadPerSec) &&
        (std::abs(right_radians) < wheelSpeedRadPerSec));
}

} // namespace ubiquity_motor
