#ifndef MCBINTERFACE_HPP
#define MCBINTERFACE_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>

#include "motor_parameters.hpp"
#include "ubiquity_motor/motor_serial.h"
#include "ubiquity_motor/motor_diagnostics.hpp"

// The gear ratio defaults for wheels shipped with Magni
static constexpr double WHEEL_GEAR_RATIO_1 = 4.294;         // Default original motor gear ratio for Magni
static constexpr double WHEEL_GEAR_RATIO_2 = 5.170;         // 2nd version standard Magni wheels gear ratio
static constexpr double TICKS_PER_RADIAN_DEFAULT = 41.004;  // For runtime use  getWheelGearRatio() * TICKS_PER_RAD_FROM_GEAR_RATIO    

#define WHEEL_GEAR_RATIO_DEFAULT WHEEL_GEAR_RATIO_1

class MCBInterface : public rclcpp::Node {
public:

    /**
     * Close off the serial port is used in a special case of suspending the motor controller
     * so that another service can load firmware or do direct MCB diagnostics
     */
    void closePort();

    /**
     * After we have given up the MCB we open serial port again using current instance of Serial
     */
    bool openPort();

    /**
     * Receive serial and act on the response from motor controller.
     * The motor controller sends unsolicited messages periodically so we must read the
     * messages to update status in near realtime
     */
    void readInputs();

    /**
     * Passes the stored joint velocity commands to the writeSpeedsInRadians function
     */
    void writeSpeeds();

    /**
     * Take in radians per sec for wheels and send in message to controller.
     * 
     * A direct write speeds that allows caller setting speeds in radians
     * This interface allows maintaining of system speed in state but override to zero
     * which is of value for such a case as ESTOP implementation
     * @param left_radians  left wheel velocity command in rad/s
     * @param right_radians right wheel velocity command in rad/s
     */
    void writeSpeedsInRadians(double left_radians, double right_radians);

    /**
     * Publish the firmware version and optional date to ROS
     */
    void publishFirmwareInfo();

    /**
     * Determine approximate battery charge percentage.
     * 
     * A battery type is defined by an array of 11 values, each corresponding to 
     * one 10% sized step from 0% to 100%. If the values fall between the steps, 
     * they are linearly interpolated to give a more accurate reading.
     * 
     * @param voltage   Battery voltage (V)
     * @param cells     Number of battery cells
     * @param type      Array of battery voltage levels for increments of 10% (per cell)
     * 
     * @return Approximate battery charge percentage
     */
    float calculateBatteryPercentage(float voltage, int cells, const float* type);

    /**
     * Determine if all wheel joint speeds are below given threshold
     * 
     * @param wheelSpeedRadPerSec   threshold value
     */
    bool areWheelSpeedsLower(double wheelSpeedRadPerSec);

    /**
     * Request the firmware version from the MCB. 
     * 
     * @note This must be followed by a readInputs() command to see the response
     */
    void requestFirmwareVersion();

    /**
     * Request the firmware date register implemented as of MIN_FW_FIRMWARE_DATE from 
     * the MCB. 
     * 
     * @note This must be followed by a readInputs() command to see the response
     */
    void requestFirmwareDate();

    /**
     * Set internal parameters (deprecated)
     */
    [[deprecated]] void setParams(ubiquity_motor::Params::FirmwareParams firmware_params);

    /**
     * Send a single parameter from FirmwareParams to the MCB. This must be called
     * for each parameter in FirmwareParams in a loop to have the full effect
     */
    void sendParams();

    /**
     * Forces next calls to sendParams() to always update each parameter
     * @note KEEP THIS IN SYNC WITH CHANGES TO sendParams()
     */
    void forcePidParamUpdates();

    /**
     * Get current battery voltage in volts
     */
    float getBatteryVoltage();

    /**
     * Transfer deadman timer to the MCB
     */
    void setDeadmanTimer(int32_t deadman);

    /**
     * Transfer deadman enable to the MCB
     */
    void setDeadzoneEnable(int32_t deadzone_enable);

    /**
     * Transfer debug LED commands to the MCB
     */
    void setDebugLeds(bool led1, bool led2);

    /**
     * Transfer hardware version to the MCB
     * 
     * @note Due to greatly limited pins on the firmware processor the host figures 
     * out the hardware rev and sends it to fw. The hardware version is 0x0000MMmm  
     * where MM is major rev like 4 and mm is minor rev like 9 for first units.
     * The 1st firmware version this is set for is 32, before it was always 1
     */
    void setHardwareVersion(int32_t hardware_version);

    /**
     * Setup the controller board threshold to put into force estop protection on 
     * boards prior to rev 5.0 with hardware support
     */
    void setEstopPidThreshold(int32_t estop_pid_threshold);

    /**
     * Setup the controller board to have estop button state detection feature enabled or not
     */
    void setEstopDetection(int32_t estop_detection);

    /**
     * Returns true if estop switch is active OR if motor power is off somehow off
     */
    bool getEstopState();

    /**
     * Setup the controller board maximum settable motor forward speed
     */
    void setMaxFwdSpeed(int32_t max_speed_fwd);

    /**
     * Setup the controller board maximum settable motor reverse speed
     */
    void setMaxRevSpeed(int32_t max_speed_rev);

    /**
     * Setup the controller board maximum PWM level allowed for a motor
     */
    void setMaxPwm(int32_t max_pwm);

    /**
     * Setup the Wheel Type. Overrides mode in use on hardware
     * This used to only be standard but THIN_WHEELS were added in Jun 2020
     */
    void setWheelType(int32_t wheel_type);

    /**
     * Setup the local Wheel gear ratio so the motor hardware layer can adjust wheel odom reporting.
     * 
     * @note This gear ratio was introduced for a new version of the standard wheels 
     * in late 2021 production. This is slightly more complex in that it is compounded 
     * with the now default 6 state encoder hw option
     */
    void setWheelGearRatio(double wheel_gear_ratio);

    /**
     * Wheel gear ratio getter
     * 
     * @note This returns local data and does not interact with the MCB
     */
    double getWheelGearRatio();

    /**
     * Encoder ticks per radian of wheel rotation getter
     * 
     * @note This returns local data and does not interact with the MCB
     */
    double getWheelTicksPerRadian();

    /**
     * Setup the Drive Type. Overrides mode in use on hardware.
     * 
     * @note This used to only be 2WD and use of THIN_WHEELS set 4WD.
     * We are not trying to decouple wheel type from drive type
     * This register always existed but was a do nothing till firmware v42
     */
    void setDriveType(int32_t drive_type);

    /**
     * Setup the PID control options
     * 
     * @note This overrides modes in use on hardware
     */
    void setPidControl(int32_t pid_control);

    /**
     * Do a one time NULL of the wheel setpoint based on current position error
     * This allows to relieve stress in static situation where wheels cannot slip to match setpoint
     */
    void nullWheelErrors();

    /**
     * Setup the Wheel direction. This allows for customer to install wheels on
     * cutom robots as they like
     * 
     * @note Overrides mode in use on hardware
     */
    void setWheelDirection(int32_t wheel_direction);

    /**
     * Getter for the wheel currents in amps
     * 
     * @param currentLeft   value to be filled with left current
     * @param currentRight  value to be filled with right current
     * 
     * @note This returns local data and does not interact with the MCB
     */
    void getMotorCurrents(double &currentLeft, double &currentRight);

    /**
     * Read the controller board option switch itself that resides on the I2C bus 
     * but is on the MCB. This call inverts the bits because a shorted option switch 
     * is a 0 where we want it as 1. If return is negative something went wrong
     */
    int  getOptionSwitch();

    /**
     * Getter for the PID control word from firmware params
     */
    int  getPidControlWord();

    /**
     * Setup the controller board option switch register which comes from the I2C 8-bit IO chip on MCB
     */
    void setOptionSwitchReg(int32_t option_switch);

    /**
     * Request the MCB system event register
     * 
     * @note This must be followed by a readInputs() command to see the response
     */
    void requestSystemEvents();

    /**
     * Setup the controller board system event register or clear bits in the register
     */
    void setSystemEvents(int32_t system_events);

    /**
     * Read the current wheel positions in radians both at once for a snapshot of position 
     * 
     * @param leftWheelPosition     value to fill with left wheel position (rad)
     * @param rightWheelPosition    value to fill with right wheel position (rad)
     */    
    void getWheelJointPositions(double &leftWheelPosition, double &rightWheelPosition);

    /**
     * Set the current wheel joing velocities in radians/sec both at once for a snapshot of velocity
     * 
     * @param leftWheelVelocity     value to set as the left wheel velocity
     * @param rightWheelVelocity    value to set as the left wheel velocity
     */
    void setWheelJointVelocities(double leftWheelVelocity, double rightWheelVelocity);

    /**
     * Publish motor state conditions
     * 
     * TODO: Move this to system inferface to make this class ROS independent
     */
    void publishMotorState();

    // Firmware revision number
    int firmware_version{0};

    // Firmware date in format 0xYYYYMMDD in hexidecimal
    int firmware_date{0};

    // Binary envoded firmware according to the MotorMessage::HwOptions enum
    int firmware_options{0};

    // Total number of firmware parameters. This is used for sendParams as modulo count
    int num_fw_params;  

    // Version number of the robot MCB
    int hardware_version;

    // Threshold for automatic estop activation based on PID outputs
    int estop_pid_threshold;

    // Max forward speed of the robot (units unknown)
    int max_speed_fwd;

    // Max reverse speed of the robot (units unknown)
    int max_speed_rev;

    // Max pwm sigal 
    int max_pwm;

    // Binary encoded system events according to the MotorMessage::SystemEvents enum
    int system_events;

    // Wheel type according to the MotorMessage::HwOptions enum
    int wheel_type;

    // Wheel gear ratio
    double wheel_gear_ratio{WHEEL_GEAR_RATIO_DEFAULT};

    // Wether the robot is using standard drive or 4-wheel drive
    int drive_type;

    // Counter for the number of times the wheel nulling method is called
    int wheel_slip_events{0};

    // Record of the wheel positions in the previous timestep
    double left_last_wheel_pos {0.0};

    // Record of the wheel positions in the previous timestep
    double right_last_wheel_pos{0.0};

    // Flag to indicate if wheel slip nulling is enabled
    bool wheel_slip_nulling{false};

    // The number of encode ticks for a single radian of wheel rotation
    double ticks_per_radian{TICKS_PER_RADIAN_DEFAULT};

private:
    // The serial interface with the MCB
    std::unique_ptr<MotorSerial> motor_serial_;

    // ROS logger
    rclcpp::Logger logger_{rclcpp::get_logger("MCBInterface")};

    // Used for keeping track of diagnostics
    MotorDiagnostics motor_diag_;

    // MessageTypes enum for refering to motor or wheel number
    enum MotorOrWheelNumber {
        Motor_M1 = 0b01,
        Motor_M2 = 0b10
    };

    // MessageTypes enum in class to avoid global namespace pollution
    enum WheelJointLocation {
        LEFT  = 0,
        RIGHT = 1
    };

    enum DriveType {
        DRIVER_TYPE_STANDARD = 0,
        DRIVER_TYPE_4WD      = 1
    } drive_type_;

    struct Joint {
        double position{};         // rad
        double velocity{};         // rad/s
        double effort{};           // Nm

        int16_t velocity_error{};  
    };
    std::array<Joint, 2> joints_;

    int16_t calculateSpeedFromRadians(double radians);
    double calculateRadiansFromTicks(int16_t ticks);

    // Input handle callbacks
    void handleReadSystemEvents(int32_t data);
    void handleReadFirmwareVersion(int32_t data);
    void handleReadFirmwareDate(int32_t data);
    void handleReadOdom(int32_t data);
    void handleReadSpeedError(int32_t data);
    void handleReadPWM(int32_t data);
    void handleReadLeftCurrent(int32_t data);
    void handleReadRightCurrent(int32_t data);
    void handleReadFirmwareOptions(int32_t data);
    void handleReadLimitReached(int32_t data);
    void handleReadBatteryVoltage(int32_t data);
};

#endif
