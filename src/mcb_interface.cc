#include "ubiquity_motor/mcb_interface.hpp"

#define LOWEST_FIRMWARE_VERSION 28
static constexpr double WHEEL_VELOCITY_NEAR_ZERO = 0.08;
static constexpr double ODOM_4WD_ROTATION_SCALE  = 1.65;
static constexpr double MOTOR_AMPS_PER_ADC_COUNT = 0.0238; // 0.1V/Amp  2.44V=1024 count so 41.97 cnt/amp

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
                    int32_t data = mm.getData();
                    
                    break;
                }
                case MotorMessage::REG_MOT_PWR_ACTIVE: {   // Starting with rev 5.0 board we can see power state
                    int32_t data = mm.getData();

                    if (data & MotorMessage::MOT_POW_ACTIVE) {
                        if (estop_motor_power_off == true) {
                            RCLCPP_WARN(logger_, "Motor power has gone from inactive to active. Most likely from ESTOP switch");
                        }
                        estop_motor_power_off = false;
                    } else {
                        if (estop_motor_power_off == false) {
                            RCLCPP_WARN(logger_, "Motor power has gone inactive. Most likely from ESTOP switch active");
                        }
                        estop_motor_power_off = true;
                    }
                    motor_diag_.estop_motor_power_off = estop_motor_power_off;  // A copy for diagnostics topic

                    std_msgs::msg::Bool estop_message;
                    estop_message.data = !estop_motor_power_off;
                    motor_power_active_pub_->publish(estop_message);
                    break;
                }

                case MotorMessage::REG_TINT_BOTH_WHLS: {   // As of v41 show time between wheel enc edges
                    int32_t data = mm.getData();
                    uint16_t leftTickSpacing = (data >> 16) & 0xffff;
                    uint16_t rightTickSpacing = data & 0xffff;
                    uint16_t tickCap = 0;    // We can cap the max value if desired

                    if ((tickCap > 0) && (leftTickSpacing  > tickCap)) { leftTickSpacing  = tickCap; }
                    if ((tickCap > 0) && (rightTickSpacing > tickCap)) { rightTickSpacing = tickCap; }

                    // Publish the two wheel tic intervals
                    std_msgs::msg::Int32 leftInterval;
                    std_msgs::msg::Int32 rightInterval;

                    leftInterval.data  = leftTickSpacing;
                    rightInterval.data = rightTickSpacing;

                    // Only publish the tic intervals when wheels are moving
                    if (data > 1) {     // Optionally show the intervals for debug
                        left_tick_interval_pub_->publish(leftInterval);
                        right_tick_interval_pub_->publish(rightInterval);

                        RCLCPP_DEBUG(logger_, "Tic Ints M1 %d [0x%x]  M2 %d [0x%x]",  
                            leftTickSpacing, leftTickSpacing, rightTickSpacing, rightTickSpacing);
                    }
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
    publishFirmwareInfo();
}

void MCBInterface::handleReadFirmwareDate(int32_t data) {
    // Firmware date is only supported as of fw version MIN_FW_FIRMWARE_DATE
    RCLCPP_INFO_ONCE(logger_, "Firmware date 0x%x (format 0xYYYYMMDD)", data);
    firmware_date = data;
    motor_diag_.firmware_date = firmware_date;

    publishFirmwareInfo();
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

    const float voltage = (float)data * fw_params.battery_voltage.multiplier +
                    fw_params.battery_voltage.offset;
                    
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

    // motor_diag_.battery_voltage = bstate.voltage;
    // motor_diag_.battery_voltage_low_level = MotorHardware::fw_params.battery_voltage.low_level;
    // motor_diag_.battery_voltage_critical = MotorHardware::fw_params.battery_voltage.critical;
}