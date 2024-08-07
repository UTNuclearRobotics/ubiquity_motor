#include "ubiquity_motor/mcb_interface.hpp"

namespace ubiquity_motor {

#define LOWEST_FIRMWARE_VERSION 28
static constexpr double WHEEL_VELOCITY_NEAR_ZERO = 0.08;
static constexpr double ODOM_4WD_ROTATION_SCALE  = 1.65;
static constexpr double MOTOR_AMPS_PER_ADC_COUNT = 0.0238; // 0.1V/Amp  2.44V=1024 count so 41.97 cnt/amp
static constexpr double VELOCITY_READ_PER_SECOND = 10.0;   // read = ticks / (100 ms), so we scale of 10 for ticks/second
static constexpr double HIGH_SPEED_RADIANS       = 1.8;    // threshold to consider wheel turning 'very fast'


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

} // namespace ubiquity_motor
