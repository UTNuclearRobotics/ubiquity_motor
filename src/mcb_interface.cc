#include "ubiquity_motor/mcb_interface.hpp"

namespace ubiquity_motor {

#define LOWEST_FIRMWARE_VERSION 28
static constexpr double WHEEL_VELOCITY_NEAR_ZERO = 0.08;
static constexpr double ODOM_4WD_ROTATION_SCALE  = 1.65;
static constexpr double MOTOR_AMPS_PER_ADC_COUNT = 0.0238; // 0.1V/Amp  2.44V=1024 count so 41.97 cnt/amp
static constexpr double VELOCITY_READ_PER_SECOND = 10.0;   // read = ticks / (100 ms), so we scale of 10 for ticks/second
static constexpr double HIGH_SPEED_RADIANS       = 1.8;    // threshold to consider wheel turning 'very fast'
static constexpr double TICKS_PER_RAD_FROM_GEAR_RATIO = 4.774556 * 2.0;
static constexpr char*  I2C_DEVICE = "/dev/i2c-1";     // This is specific to default Magni I2C port on host
static constexpr uint8_t I2C_PCF8574_8BIT_ADDR = 0x40;


void MCBInterface::closePort() {
    if (motor_serial_)
        motor_serial_->closePort();
    else
        RCLCPP_WARN(logger_, "Cannot close port, motor serial has not been initialized!");
}

bool MCBInterface::openPort() {
    if (motor_serial_) {
        bool success = motor_serial_->openPort();
        if (!success)
            RCLCPP_WARN(logger_, "Failed on open MCB port!");
        return success;
    } else {
        RCLCPP_WARN(logger_, "Cannot open port, motor serial has not been initialized!");
        return false;
    }
}

// ================================================================================================
// Read data from MCB
// ================================================================================================

void MCBInterface::readInputs() {
    while (motor_serial_->commandAvailable()) {
        MotorMessage mm;
        mm = motor_serial_->receiveCommand();
        if (mm.getType() == MotorMessage::TYPE_RESPONSE) {
            switch (mm.getRegister()) {

                case MotorMessage::REG_SYSTEM_EVENTS:
                    handleReadSystemEvents(mm.getData());
                    break;

                case MotorMessage::REG_FIRMWARE_VERSION:
                    handleReadFirmwareVersion(mm.getData());
                    break;

                case MotorMessage::REG_FIRMWARE_DATE:
                    handleReadFirmwareDate(mm.getData());
                    break;

                case MotorMessage::REG_BOTH_ODOM:
                    handleReadOdom(mm.getData());
                    break;

                case MotorMessage::REG_BOTH_ERROR:
                    handleReadSpeedError(mm.getData());
                    break;

                case MotorMessage::REG_PWM_BOTH_WHLS:
                    handleReadPWM(mm.getData());
                    break;

                case MotorMessage::REG_LEFT_CURRENT:
                    handleReadLeftCurrent(mm.getData());
                    break;

                case MotorMessage::REG_RIGHT_CURRENT:
                    handleReadRightCurrent(mm.getData());
                    break;

                case MotorMessage::REG_HW_OPTIONS:
                    handleReadFirmwareOptions(mm.getData());
                    break;

                case MotorMessage::REG_LIMIT_REACHED:
                    handleReadLimitReached(mm.getData());
                    break;

                case MotorMessage::REG_BATTERY_VOLTAGE: {
                    handleReadBatteryVoltage(mm.getData());
                    break;
                }

                case MotorMessage::REG_MOT_PWR_ACTIVE: {
                    handleReadMotorPowerActive(mm.getData());
                    break;
                }

                case MotorMessage::REG_TINT_BOTH_WHLS: { 
                   handleReadEncodeTimeInc(mm.getData());
                   break;
                }

                default:
                    break;
            }
        }
    }
}

void MCBInterface::handleReadSystemEvents(int32_t data) {
    if (data & MotorMessage::SYS_EVENT_POWERON) {
        RCLCPP_WARN(logger_, "Firmware System Event for PowerOn transition");
        system_events = data;
    }
}

void MCBInterface::handleReadFirmwareVersion(int32_t data) {
    if (data < LOWEST_FIRMWARE_VERSION) {
        RCLCPP_FATAL(logger_, "Firmware version %d, expect %d or above",
                    data, LOWEST_FIRMWARE_VERSION);
        throw std::runtime_error("Firmware version too low");
    } else {
        RCLCPP_INFO_ONCE(logger_, "Firmware version %d", data);
        firmware_version = data;
        motor_diag_.firmware_version = firmware_version;
    }
    // publishFirmwareInfo();
}

void MCBInterface::handleReadFirmwareDate(int32_t data) {
    // Firmware date is only supported as of fw version MIN_FW_FIRMWARE_DATE
    RCLCPP_INFO_ONCE(logger_, "Firmware date 0x%x (format 0xYYYYMMDD)", data);
    firmware_date = data;
    motor_diag_.firmware_date = firmware_date;

    // publishFirmwareInfo();
}

void MCBInterface::handleReadOdom(int32_t data) {
    /* 
    * ODOM messages from the MCB tell us how far wheels have rotated
    *
    * It is here we keep track of wheel joint position 
    * The odom counts from the MCB are the incremental number of ticks since last report
    *  WARNING: IF WE LOOSE A MESSAGE WE DRIFT FROM REAL POSITION
    */
    const int32_t odom = data;
    // RCLCPP_ERROR(logger_, "odom signed %d", odom);
    const int16_t odomLeft = (odom >> 16) & 0xffff;
    const int16_t odomRight = odom & 0xffff;

    // Due to extreme wheel slip that is required to turn a 4WD robot we are doing a scale factor.
    // When doing a rotation on the 4WD robot that is in place where linear velocity is zero
    // we will adjust the odom values for wheel joint rotation using the scale factor.
    double odom4wdRotationScale = 1.0;

    // Determine if we are rotating then set a scale to account for rotational wheel slip
    int leftDir  = (joints_[LEFT].velocity >= 0.0) ? 1 : -1;
    int rightDir = (joints_[RIGHT].velocity >= 0.0) ? 1 : -1;
    // int is4wdMode = (fw_params.hw_options & MotorMessage::OPT_DRIVE_TYPE_4WD);
    if (
        // Is this in 4wd robot mode
        (drive_type_ == DRIVER_TYPE_4WD)

        // Do the joints have Different rotational directions
        && ((leftDir + rightDir) == 0)

        // Are Both joints not near joint velocity of 0
        && ((std::abs(joints_[LEFT].velocity) > WHEEL_VELOCITY_NEAR_ZERO) && (std::abs(joints_[RIGHT].velocity) > WHEEL_VELOCITY_NEAR_ZERO))

        // Is the difference of the two absolute values of the joint velocities near zero
        && ((std::abs(joints_[LEFT].velocity) - std::abs(joints_[RIGHT].velocity)) < WHEEL_VELOCITY_NEAR_ZERO) )  
    {
        odom4wdRotationScale = ODOM_4WD_ROTATION_SCALE;
        rclcpp::Clock clock{};
        RCLCPP_INFO_THROTTLE(logger_, clock, 1.0, "ROTATIONAL_SCALING_ACTIVE: odom4wdRotationScale = %4.2f [%4.2f, %4.2f] [%d,%d] opt 0x%lx 4wd=%d",
                odom4wdRotationScale, joints_[LEFT].velocity, joints_[RIGHT].velocity, leftDir, rightDir, firmware_options, (drive_type_ == DRIVER_TYPE_4WD));
    } else {
        if (fabs(joints_[LEFT].velocity) > WHEEL_VELOCITY_NEAR_ZERO) {
            RCLCPP_DEBUG(logger_, "odom4wdRotationScale = %4.2f [%4.2f, %4.2f] [%d,%d] opt 0x%lx 4wd=%d",
                odom4wdRotationScale, joints_[LEFT].velocity, joints_[RIGHT].velocity, leftDir, rightDir, firmware_options, (drive_type_ == DRIVER_TYPE_4WD));
        }
    }

    // Add or subtract from position in radians using the incremental odom value
    joints_[LEFT].position  += (odomLeft / (ticks_per_radian * odom4wdRotationScale));
    joints_[RIGHT].position += (odomRight / (ticks_per_radian * odom4wdRotationScale));

    motor_diag_.odom_update_status.tick(); // Let diag know we got odom
}

void MCBInterface::handleReadSpeedError(int32_t data) {
    int32_t speed = data;
    joints_[LEFT].velocity_error  = (speed >> 16) & 0xffff;
    joints_[RIGHT].velocity_error = speed & 0xffff;
}

void MCBInterface::handleReadPWM(int32_t data) {
    int32_t bothPwm = data;
    motor_diag_.motorPwmDriveLeft  = (bothPwm >> 16) & 0xffff;
    motor_diag_.motorPwmDriveRight = bothPwm & 0xffff;
}


void MCBInterface::handleReadLeftCurrent(int32_t data) {
    // Motor current is an absolute value and goes up from a nominal count of near 1024
    // So we subtract a nominal offset then multiply count * scale factor to get amps
    int32_t data = data & 0xffff;
    motor_diag_.motorCurrentLeft =
        (double)(data - motor_diag_.motorAmpsZeroAdcCount) * MOTOR_AMPS_PER_ADC_COUNT;
}

void MCBInterface::handleReadRightCurrent(int32_t data) {
    // Motor current is an absolute value and goes up from a nominal count of near 1024
    // So we subtract a nominal offset then multiply count * scale factor to get amps
    int32_t data = data & 0xffff;
    motor_diag_.motorCurrentRight =
        (double)(data - motor_diag_.motorAmpsZeroAdcCount) * MOTOR_AMPS_PER_ADC_COUNT;
}

void MCBInterface::handleReadFirmwareOptions(int32_t data) {
    // Enable or disable hardware options reported from firmware
    motor_diag_.firmware_options = data;

    // Set radians per encoder tic based on encoder specifics
    if (data & MotorMessage::OPT_ENC_6_STATE) {
        RCLCPP_WARN_ONCE(logger_, "Encoder Resolution: 'Enhanced'");
        firmware_options |= MotorMessage::OPT_ENC_6_STATE;
        ticks_per_radian = getWheelTicksPerRadian();
    } else {
        RCLCPP_WARN_ONCE(logger_, "Encoder Resolution: 'Standard'");
        firmware_options &= ~MotorMessage::OPT_ENC_6_STATE;
        ticks_per_radian  = getWheelTicksPerRadian() / (double)(2.0);
    }

    if (data & MotorMessage::OPT_WHEEL_TYPE_THIN) {
        RCLCPP_WARN_ONCE(logger_, "Wheel type is: 'thin'");
        firmware_options |= MotorMessage::OPT_WHEEL_TYPE_THIN;
    } else {
        RCLCPP_WARN_ONCE(logger_, "Wheel type is: 'standard'");
        firmware_options &= ~MotorMessage::OPT_WHEEL_TYPE_THIN;
    }

    if (data & MotorMessage::OPT_DRIVE_TYPE_4WD) {
        RCLCPP_WARN_ONCE(logger_, "Drive type is: '4wd'");
        firmware_options |= MotorMessage::OPT_DRIVE_TYPE_4WD;
    } else {
        RCLCPP_WARN_ONCE(logger_, "Drive type is: '2wd'");
        firmware_options &= ~MotorMessage::OPT_DRIVE_TYPE_4WD;
    }

    if (data & MotorMessage::OPT_WHEEL_DIR_REVERSE) {
        RCLCPP_WARN_ONCE(logger_, "Wheel direction is: 'reverse'");
        firmware_options |= MotorMessage::OPT_WHEEL_DIR_REVERSE;
    } else {
        RCLCPP_WARN_ONCE(logger_, "Wheel direction is: 'standard'");
        firmware_options &= ~MotorMessage::OPT_WHEEL_DIR_REVERSE;
    }
}

void MCBInterface::handleReadLimitReached(int32_t data) {
    if (data & MotorMessage::LIM_M1_PWM) {
        RCLCPP_WARN(logger_, "left PWM limit reached");
            motor_diag_.left_pwm_limit = true;
    }
    if (data & MotorMessage::LIM_M2_PWM) {
        RCLCPP_WARN(logger_, "right PWM limit reached");
            motor_diag_.right_pwm_limit = true;
    }
    if (data & MotorMessage::LIM_M1_INTEGRAL) {
        RCLCPP_DEBUG(logger_, "left Integral limit reached");
            motor_diag_.left_integral_limit = true;
    }
    if (data & MotorMessage::LIM_M2_INTEGRAL) {
        RCLCPP_DEBUG(logger_, "right Integral limit reached");
            motor_diag_.right_integral_limit = true;
    }
    if (data & MotorMessage::LIM_M1_MAX_SPD) {
        RCLCPP_WARN(logger_, "left Maximum speed reached");
            motor_diag_.left_max_speed_limit = true;
    }
    if (data & MotorMessage::LIM_M2_MAX_SPD) {
        RCLCPP_WARN(logger_, "right Maximum speed reached");
            motor_diag_.right_max_speed_limit = true;
    }
    if (data & MotorMessage::LIM_PARAM_LIMIT) {
        RCLCPP_WARN_ONCE(logger_, "parameter limit in firmware");
            motor_diag_.param_limit_in_firmware = true;
    }
}

void MCBInterface::handleReadBatteryVoltage(int32_t data) {

    const float voltage = (float)data * fw_params_->battery_voltage.multiplier +
                    fw_params_->battery_voltage.offset;
                    
    // sensor_msgs::msg::BatteryState bstate;
    // bstate.voltage = (float)data * fw_params.battery_voltage.multiplier +
    //                 fw_params.battery_voltage.offset;
    // bstate.current = std::numeric_limits<float>::quiet_NaN();
    // bstate.charge = std::numeric_limits<float>::quiet_NaN();
    // bstate.capacity = std::numeric_limits<float>::quiet_NaN();
    // bstate.design_capacity = std::numeric_limits<float>::quiet_NaN();

    // // Hardcoded to a sealed lead acid 12S battery, but adjustable for future use
    // bstate.percentage = calculateBatteryPercentage(bstate.voltage, 12, SLA_AGM);
    // bstate.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    // bstate.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    // bstate.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    // battery_state_pub_->publish(bstate);

    motor_diag_.battery_voltage = voltage;
    motor_diag_.battery_voltage_low_level = fw_params_->battery_voltage.low_level;
    motor_diag_.battery_voltage_critical = fw_params_->battery_voltage.critical;
}

void MCBInterface::handleReadMotorPowerActive(int32_t data) {
    // Starting with rev 5.0 board we can see power state
    if (data & MotorMessage::MOT_POW_ACTIVE) {
        if (estop_motor_power_off_ == true) {
            RCLCPP_WARN(logger_, "Motor power has gone from inactive to active. Most likely from ESTOP switch");
        }
        estop_motor_power_off_ = false;
    } else {
        if (estop_motor_power_off_ == false) {
            RCLCPP_WARN(logger_, "Motor power has gone inactive. Most likely from ESTOP switch active");
        }
        estop_motor_power_off_ = true;
    }
    motor_diag_.estop_motor_power_off = estop_motor_power_off_;  // A copy for diagnostics topic

    // std_msgs::msg::Bool estop_message;
    // estop_message.data = !estop_motor_power_off;
    // motor_power_active_pub_->publish(estop_message);
}

void MCBInterface::handleReadEncodeTimeInc(int32_t data) {
    // As of v41 show time between wheel enc edges
    leftTickSpacing = (data >> 16) & 0xffff;
    rightTickSpacing = data & 0xffff;
    uint16_t tickCap = 0;    // We can cap the max value if desired

    if ((tickCap > 0) && (leftTickSpacing  > tickCap)) { leftTickSpacing  = tickCap; }
    if ((tickCap > 0) && (rightTickSpacing > tickCap)) { rightTickSpacing = tickCap; }

    // Publish the two wheel tic intervals
    // std_msgs::msg::Int32 leftInterval;
    // std_msgs::msg::Int32 rightInterval;

    // leftInterval.data  = leftTickSpacing;
    // rightInterval.data = rightTickSpacing;

    // Only publish the tic intervals when wheels are moving
    if (data > 1) {     // Optionally show the intervals for debug
        // left_tick_interval_pub_->publish(leftInterval);
        // right_tick_interval_pub_->publish(rightInterval);

        RCLCPP_DEBUG(logger_, "Tic Ints M1 %d [0x%x]  M2 %d [0x%x]",  
            leftTickSpacing, leftTickSpacing, rightTickSpacing, rightTickSpacing);
    }
}

// ================================================================================================
// Write data to MCB
// ================================================================================================

void MCBInterface::writeSpeedsInRadians(double left_radians, double right_radians) {
    MotorMessage both;
    both.setRegister(MotorMessage::REG_BOTH_SPEED_SET);
    both.setType(MotorMessage::TYPE_WRITE);

    // We are going to implement a message when robot is moving very fast or rotating very fast
    if (((left_radians / VELOCITY_READ_PER_SECOND)  > HIGH_SPEED_RADIANS) || 
        ((right_radians / VELOCITY_READ_PER_SECOND) > HIGH_SPEED_RADIANS)) {
        RCLCPP_WARN(logger_, "Wheel rotation at high radians per sec.  Left %f rad/s Right %f rad/s",
            left_radians, right_radians);
    }

    int16_t left_speed  = calculateSpeedFromRadians(left_radians);
    int16_t right_speed = calculateSpeedFromRadians(right_radians);

    // The masking with 0x0000ffff is necessary for handling -ve numbers
    int32_t data = (left_speed << 16) | (right_speed & 0x0000ffff);
    both.setData(data);
    motor_serial_->transmitCommand(both);

    // std_msgs::msg::Int32 smsg;
    // smsg.data = left_speed;


    // RCLCPP_ERROR(nh_->get_logger(), "velocity_command %f rad/s %f rad/s",
    // joints_[WheelJointLocation::Left].velocity_command, joints_[WheelJointLocation::Right].velocity_command);
    // joints_[LEFT_WHEEL_JOINT].velocity_command, joints_[RIGHT_WHEEL_JOINT].velocity_command);
    // RCLCPP_ERROR(nh_->get_logger(), "SPEEDS %d %d", left.getData(), right.getData());
}

float MCBInterface::calculateBatteryPercentage(float voltage, int cells, const float* type) {
    float onecell = voltage / (float)cells;

    if(onecell >= type[10])
        return 1.0;
    else if(onecell <= type[0])
        return 0.0;

    int upper = 0;
    int lower = 0;

    for(int i = 0; i < 11; i++){
        if(onecell > type[i]){
            lower = i;
        }else{
            upper = i;
            break;
        }
    }

    float deltavoltage = type[upper] - type[lower];
    float between_percent = (onecell - type[lower]) / deltavoltage;

    return (float)lower * 0.1 + between_percent * 0.1;
}

void MCBInterface::requestFirmwareVersion() {
    MotorMessage fw_version_msg;
    fw_version_msg.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
    fw_version_msg.setType(MotorMessage::TYPE_READ);
    fw_version_msg.setData(0);
    motor_serial_->transmitCommand(fw_version_msg);
}

void MCBInterface::requestFirmwareDate() {
    MotorMessage fw_date_msg;
    fw_date_msg.setRegister(MotorMessage::REG_FIRMWARE_DATE);
    fw_date_msg.setType(MotorMessage::TYPE_READ);
    fw_date_msg.setData(0);
    motor_serial_->transmitCommand(fw_date_msg);
}

void MCBInterface::requestSystemEvents() {
    MotorMessage sys_event_msg;
    sys_event_msg.setRegister(MotorMessage::REG_SYSTEM_EVENTS);
    sys_event_msg.setType(MotorMessage::TYPE_READ);
    sys_event_msg.setData(0);
    motor_serial_->transmitCommand(sys_event_msg);
}

void MCBInterface::setDeadmanTimer(int32_t data) {
    RCLCPP_ERROR(logger_, "setting deadman to %d", (int)data);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DEADMAN);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(data);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setDeadzoneEnable(int32_t deadzone_enable) {
    RCLCPP_ERROR(logger_, "setting deadzone enable to %d", (int)deadzone_enable);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DEADZONE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(deadzone_enable);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setDebugLeds(bool led1_val, bool led2_val) {
    std::vector<MotorMessage> commands;

    MotorMessage led1;
    led1.setRegister(MotorMessage::REG_LED_1);
    led1.setType(MotorMessage::TYPE_WRITE);
    if (led1_val) {
        led1.setData(0x00000001);
    } else {
        led1.setData(0x00000000);
    }
    commands.push_back(led1);

    MotorMessage led2;
    led2.setRegister(MotorMessage::REG_LED_2);
    led2.setType(MotorMessage::TYPE_WRITE);
    if (led2_val) {
        led2.setData(0x00000001);
    } else {
        led2.setData(0x00000000);
    }
    commands.push_back(led2);

    motor_serial_->transmitCommands(commands);
}

void MCBInterface::setHardwareVersion(int32_t hardware_version) {
    RCLCPP_INFO(logger_, "setting hardware_version to %x", (int)hardware_version);
    this->hardware_version = hardware_version;
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_HARDWARE_VERSION);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(hardware_version);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setEstopPidThreshold(int32_t estop_pid_threshold) {
    RCLCPP_INFO(logger_, "setting Estop PID threshold to %d", (int)estop_pid_threshold);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_PID_MAX_ERROR);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(estop_pid_threshold);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setEstopDetection(int32_t estop_detection) {
    RCLCPP_INFO(logger_, "setting estop button detection to %x", (int)estop_detection);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_ESTOP_ENABLE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(estop_detection);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setMaxFwdSpeed(int32_t max_speed_fwd) {
    RCLCPP_INFO(logger_, "setting max motor forward speed to %d", (int)max_speed_fwd);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_SPEED_FWD);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_speed_fwd);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setMaxRevSpeed(int32_t max_speed_rev) {
    RCLCPP_INFO(logger_, "setting max motor reverse speed to %d", (int)max_speed_rev);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_SPEED_REV);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_speed_rev);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setMaxPwm(int32_t max_pwm) {
    RCLCPP_INFO(logger_, "setting max motor PWM to %x", (int)max_pwm);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_PWM);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_pwm);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setWheelType(int32_t new_wheel_type) {
    MotorMessage mm;
    switch(new_wheel_type) {
        case MotorMessage::OPT_WHEEL_TYPE_STANDARD:
        case MotorMessage::OPT_WHEEL_TYPE_THIN:
            RCLCPP_INFO_ONCE(logger_, "setting MCB wheel type %d", (int)new_wheel_type);
            wheel_type_ = new_wheel_type;
            mm.setRegister(MotorMessage::REG_WHEEL_TYPE);
            mm.setType(MotorMessage::TYPE_WRITE);
            mm.setData(wheel_type_);
            motor_serial_->transmitCommand(mm);
            break;
        default:
            RCLCPP_ERROR(logger_, "Illegal MCB wheel type 0x%x will not be set!", (int)new_wheel_type);
    }
}

void MCBInterface::setWheelGearRatio(double new_wheel_gear_ratio) {
    // This gear ratio is not used by the firmware so it is a simple state element in this module
    wheel_gear_ratio_ = new_wheel_gear_ratio;
    ticks_per_radian  = getWheelTicksPerRadian();   // Need to also reset ticks_per_radian
    if ((fw_params_->hw_options & MotorMessage::OPT_ENC_6_STATE) == 0) {
        ticks_per_radian = ticks_per_radian / (double)(2.0);   // 3 state was half
    }
    RCLCPP_INFO(logger_, "Setting Wheel gear ratio to %6.4f and tics_per_radian to %6.4f",
        wheel_gear_ratio_, ticks_per_radian);
}

void MCBInterface::setDriveType(int32_t drive_type) {
    RCLCPP_INFO_ONCE(logger_, "Setting MCB drive type %d", (int)drive_type);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DRIVE_TYPE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(drive_type);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setPidControl(int32_t pid_control_word) {
    RCLCPP_INFO_ONCE(logger_, "setting MCB pid control word to 0x%x", (int)pid_control_word);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_PID_CONTROL);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(pid_control_word);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::nullWheelErrors() {
    RCLCPP_DEBUG(logger_, "Nulling MCB wheel errors using current wheel positions");
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_WHEEL_NULL_ERR);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(MotorOrWheelNumber::Motor_M1|MotorOrWheelNumber::Motor_M2);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setWheelDirection(int32_t wheel_direction) {
    RCLCPP_INFO(logger_, "setting MCB wheel direction to %d", (int)wheel_direction);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_WHEEL_DIR);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(wheel_direction);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setOptionSwitchReg(int32_t option_switch_bits) {
    RCLCPP_INFO(logger_, "setting MCB option switch register to 0x%x", (int)option_switch_bits);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_OPTION_SWITCH);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(option_switch_bits);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::setSystemEvents(int32_t system_events) {
    RCLCPP_INFO(logger_, "setting MCB system event register to %d", (int)system_events);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_SYSTEM_EVENTS);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(system_events);
    motor_serial_->transmitCommand(mm);
}

void MCBInterface::sendParams() {
    std::vector<MotorMessage> commands;

    if (fw_params_->pid.proportional != prev_fw_params_.pid.proportional) {
        RCLCPP_WARN(logger_, "Setting PidParam P to %ld", fw_params_->pid.proportional);
        prev_fw_params_.pid.proportional = fw_params_->pid.proportional;
        motor_diag_.fw_pid_proportional = fw_params_->pid.proportional;
        MotorMessage p;
        p.setRegister(MotorMessage::REG_PARAM_P);
        p.setType(MotorMessage::TYPE_WRITE);
        p.setData(fw_params_->pid.proportional);
        commands.push_back(p);
    }

    if (fw_params_->pid.integral != prev_fw_params_.pid.integral) {
        RCLCPP_WARN(logger_, "Setting PidParam I to %ld", fw_params_->pid.integral);
        prev_fw_params_.pid.integral = fw_params_->pid.integral;
        motor_diag_.fw_pid_integral = fw_params_->pid.integral;
        MotorMessage i;
        i.setRegister(MotorMessage::REG_PARAM_I);
        i.setType(MotorMessage::TYPE_WRITE);
        i.setData(fw_params_->pid.integral);
        commands.push_back(i);
    }

    if (fw_params_->pid.derivative != prev_fw_params_.pid.derivative) {
        RCLCPP_WARN(logger_, "Setting PidParam D to %ld", fw_params_->pid.derivative);
        prev_fw_params_.pid.derivative = fw_params_->pid.derivative;
        motor_diag_.fw_pid_derivative = fw_params_->pid.derivative;
        MotorMessage d;
        d.setRegister(MotorMessage::REG_PARAM_D);
        d.setType(MotorMessage::TYPE_WRITE);
        d.setData(fw_params_->pid.derivative);
        commands.push_back(d);
    }

    if ((motor_diag_.firmware_version >= MIN_FW_PID_V_TERM) &&
        fw_params_->pid.velocity != prev_fw_params_.pid.velocity) {
        RCLCPP_WARN(logger_, "Setting PidParam V to %f", fw_params_->pid.velocity);
        prev_fw_params_.pid.velocity = fw_params_->pid.velocity;
        motor_diag_.fw_pid_velocity = fw_params_->pid.velocity;
        MotorMessage v;
        v.setRegister(MotorMessage::REG_PARAM_V);
        v.setType(MotorMessage::TYPE_WRITE);
        v.setData(fw_params_->pid.velocity);
        commands.push_back(v);
    }

    if (fw_params_->pid.denominator != prev_fw_params_.pid.denominator) {
        RCLCPP_WARN(logger_, "Setting PidParam Denominator to %ld", fw_params_->pid.denominator);
        prev_fw_params_.pid.denominator = fw_params_->pid.denominator;
        motor_diag_.fw_pid_denominator = fw_params_->pid.denominator;
        MotorMessage denominator;
        denominator.setRegister(MotorMessage::REG_PARAM_C);
        denominator.setType(MotorMessage::TYPE_WRITE);
        denominator.setData(fw_params_->pid.denominator);
        commands.push_back(denominator);
    }

    if (fw_params_->pid.moving_buffer_size !=
            prev_fw_params_.pid.moving_buffer_size) {
        RCLCPP_WARN(logger_, "Setting PidParam D window to %ld", fw_params_->pid.moving_buffer_size);
        prev_fw_params_.pid.moving_buffer_size =
            fw_params_->pid.moving_buffer_size;
        motor_diag_.fw_pid_moving_buffer_size = fw_params_->pid.moving_buffer_size;
        MotorMessage winsize;
        winsize.setRegister(MotorMessage::REG_MOVING_BUF_SIZE);
        winsize.setType(MotorMessage::TYPE_WRITE);
        winsize.setData(fw_params_->pid.moving_buffer_size);
        commands.push_back(winsize);
    }

    if (fw_params_->max_pwm != prev_fw_params_.max_pwm) {
        RCLCPP_WARN(logger_, "Setting PidParam max_pwm to %ld", fw_params_->max_pwm);
        prev_fw_params_.max_pwm = fw_params_->max_pwm;
        motor_diag_.fw_max_pwm = fw_params_->max_pwm;
        MotorMessage maxpwm;
        maxpwm.setRegister(MotorMessage::REG_MAX_PWM);
        maxpwm.setType(MotorMessage::TYPE_WRITE);
        maxpwm.setData(fw_params_->max_pwm);
        commands.push_back(maxpwm);
    }

    if (fw_params_->pid.control != prev_fw_params_.pid.control) {
        RCLCPP_WARN(logger_, "Setting PidParam pid_control to %ld", fw_params_->pid.control);
        prev_fw_params_.pid.control = fw_params_->pid.control;
        motor_diag_.fw_pid_control = fw_params_->pid.control;
        MotorMessage mmsg;
        mmsg.setRegister(MotorMessage::REG_PID_CONTROL);
        mmsg.setType(MotorMessage::TYPE_WRITE);
        mmsg.setData(fw_params_->pid.control);
        commands.push_back(mmsg);
    }

    // Only send one register at a time to avoid overwhelming serial comms
    for (const MotorMessage& command : commands) {
        motor_serial_->transmitCommand(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// ================================================================================================
// Utility functions
// ================================================================================================

void MCBInterface::forcePidParamUpdates() {
    // Reset each of the flags that causes parameters to be  sent to MCB by sendParams()
    prev_fw_params_.pid.proportional = -1;
    prev_fw_params_.pid.integral = -1;
    prev_fw_params_.pid.derivative = -1;
    prev_fw_params_.pid.velocity = -1;
    prev_fw_params_.pid.denominator = -1;
    prev_fw_params_.pid.moving_buffer_size = -1;
    prev_fw_params_.max_pwm = -1;
    prev_fw_params_.pid.control = 1;
}

float MCBInterface::getBatteryVoltage() {
    return motor_diag_.battery_voltage; 
}

bool MCBInterface::getEstopState() {
    return estop_motor_power_off_;
}

double MCBInterface::getWheelGearRatio() {
    return wheel_gear_ratio_;
}

double MCBInterface::getWheelTicksPerRadian() {
    return this->getWheelGearRatio() * TICKS_PER_RAD_FROM_GEAR_RATIO;
}

std::pair<double, double> MCBInterface::getMotorCurrents() {
    return {
        motor_diag_.motorCurrentLeft,
        motor_diag_.motorCurrentRight
    };
}

int MCBInterface::getOptionSwitch() {
    uint8_t buf[16];
    int retBits = 0;
    RCLCPP_INFO(logger_, "reading MCB option switch on the I2C bus");
    int retCount = i2c_BufferRead(I2C_DEVICE, I2C_PCF8574_8BIT_ADDR, &buf[0], -1, 1, logger_);
    if (retCount < 0) {
        RCLCPP_ERROR(logger_, "Error %d in reading MCB option switch at 8bit Addr 0x%x",
            retCount, I2C_PCF8574_8BIT_ADDR);
        retBits = retCount;
    } else if (retCount != 1) {
        RCLCPP_ERROR(logger_, "Cannot read byte from MCB option switch at 8bit Addr 0x%x", I2C_PCF8574_8BIT_ADDR);
        retBits = -1;
    } else {
        retBits = (0xff) & ~buf[0];
    }

    return retBits;
}

int MCBInterface::getPidControlWord() {
    return motor_diag_.fw_pid_control;
}

std::pair<double, double> MCBInterface::getWheelJointPositions() {
    return {joints_[WheelJointLocation::LEFT].position,
            joints_[WheelJointLocation::RIGHT].position};
}

std::pair<double, double> MCBInterface::setWheelJointVelocities() {
    return {joints_[WheelJointLocation::LEFT].velocity,
            joints_[WheelJointLocation::RIGHT].velocity};
}

// ================================================================================================
// Private Methods
// ================================================================================================

int16_t MCBInterface::calculateSpeedFromRadians(double radians) {
    // The firmware accepts same units for speed value
    // and will deal with it properly depending on encoder handling in use
    const double encoderFactor = (fw_params_->hw_options & MotorMessage::OPT_ENC_6_STATE) ? 0.5 : 1.0;
    const double speedFloat = encoderFactor * radians * ((getWheelTicksPerRadian() * 4.0) / VELOCITY_READ_PER_SECOND);

    return static_cast<int16_t>(std::round(speedFloat));
}

double MCBInterface::calculateRadiansFromTicks(int16_t ticks) {
    return (static_cast<double>(ticks) * VELOCITY_READ_PER_SECOND) / (getWheelTicksPerRadian() * 4.0);
}


} // namespace ubiquity_motor




// ================================================================================================
// I2C Interface
// ================================================================================================

// To access I2C we need some system includes
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <sys/ioctl.h>

static int i2c_BufferRead(const char *i2cDevFile, uint8_t i2c8bitAddr,
                          uint8_t *pBuffer, int16_t chipRegAddr, uint16_t NumBytesToRead, rclcpp::Logger logger)
{
   int bytesRead = 0;
   int retCode   = 0;

    int fd;                                         // File descrition
    int  address   = i2c8bitAddr >> 1;              // Address of the I2C device
    uint8_t buf[8];                                 // Buffer for data being written to the i2c device

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {      // Open port for reading and writing
      retCode = -2;
      RCLCPP_ERROR(logger, "Cannot open I2C def of %s with error %s", i2cDevFile, strerror(errno));
      goto exitWithNoClose;
    }

    // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    if (ioctl(fd, I2C_SLAVE, address) != 0) {        // Set the port options and addr of the dev
      retCode = -3;
      RCLCPP_ERROR(logger, "Failed to get bus access to I2C device %s!  ERROR: %s", i2cDevFile, strerror(errno));
      goto exitWithFileClose;
    }

    if (chipRegAddr < 0) {     // Suppress reg address if negative value was used
      buf[0] = (uint8_t)(chipRegAddr);          // Internal chip register address
      if ((write(fd, buf, 1)) != 1) {           // Write both bytes to the i2c port
        retCode = -4;
        goto exitWithFileClose;
      }
    }

    bytesRead = read(fd, pBuffer, NumBytesToRead);
    if (bytesRead != NumBytesToRead) {      // verify the number of bytes we requested were read
      retCode = -9;
      goto exitWithFileClose;
    }
    retCode = bytesRead;

  exitWithFileClose:
    close(fd);

  exitWithNoClose:

  return retCode;
}
