#ifndef ECU_H
#define ECU_H

#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <math.h>
#include <mutex>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/signal.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lois_ecu/msg/runtime_parameters.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include <lois_ecu/inipp.h>

#define MESSAGE_INSTRUCTIONBYTES 1
#define MESSAGE_DATABYTES 4
#define MESSAGE_TIMEOUT_MS 150

using namespace std::chrono_literals;

/**
 @brief ECU main class. Handles all ECU communication.
 **/
class ECU : public rclcpp::Node
{
  public:
    /**
     @brief Instruction headers for ECU communication protocol
     **/
    enum Commands
    {
        DUTYCYCLE = 0x01,
        RPM = 0x02,
        LED = 0x03,
        RECORDRPM = 0x04,
        EMSTOP = 0x05,
        LEFT_KP = 0x06,
        LEFT_TN = 0x07,
        LEFT_TD = 0x08,
        RIGHT_KP = 0x09,
        RIGHT_TN = 0x0a,
        RIGHT_TD = 0x0b,
        RECORDHEADING = 0x0c,
        PERIODIC_LATLON = 0x0d,
        PERIODIC_TIME = 0x0e,
        PERIODIC_DATE = 0x0f,
        PERIODIC_HEADING = 0x10,
        INSTRUCTION_TERMINALMODE = 0x11,
        PERIODIC_ENCODERS = 0x12,
        LEFT_CORRFAC_SHORT = 0x13,
        LEFT_CORRFAC_LONG = 0x14,
        RIGHT_CORRFAC_SHORT = 0x15,
        RIGHT_CORRFAC_LONG = 0x16,
        RECORDTORQUE = 0x17,
        RESET = 0x18,
        SHIT = 0x19,
        PERIODIC_ODOM = 0x20,
        LEFT_KA = 0x21,
        RIGHT_KA = 0x22,
        RPM_LOWPASS = 0x23
    };

    /**
     @brief Container for rpm realtime measurements from ECU record buffer
     **/
    typedef struct RPMData_t
    {
        std::queue<uint16_t> left;
        std::queue<uint16_t> right;
    } RPMData_t;

    /**
     @brief Container for compass heading realtime measurements from ECU record buffer
     **/
    typedef struct HeadingData_t
    {
        std::queue<short> heading;
    } HeadingData_t;

    /**
      @brief Container for GPS data
     **/
    typedef struct GPSData_t
    {
        float latitude, longitude;
    } GPSData_t;

    /**
     @brief Container for time measurements and synchronisation
     **/
    typedef struct Timestamp_t
    {
        struct timeval start, stop;
    } Timestamp_t;

    /**
     @brief Container for wheel odometry data. Includes raw ticks, position, speed and time.
     **/
    typedef struct OdometryData_t
    {
        int leftTicks, rightTicks; // raw ticks
        float x, y, o;             // x, y and yaw
        float dx, dy, w;           // speed
        uint32_t time, dt;         // timestamp and duration in ms
    } OdometryData_t;

    /**
     @brief Container for all runtime parameters, e.g. rpm controller parameters, terminal mode,
     message periods, ... Usually loaded from .ini File
     **/
    typedef struct RuntimeParameters_t
    {
        int terminalMode;                                               // Terminal mode
        int rpmctrlEnable;                                              // Enable rpm control
        float kaLeft, kpLeft, tnLeft, tdLeft, corrShortLeft, corrLongLeft;      // left rpm control
        float kaRight, kpRight, tnRight, tdRight, corrShortRight, corrLongRight; // right rpm control
        float rpmLowpass;
        int periodLatLon, periodTime, periodDate, periodHeading, periodEncoders,
            periodOdometry; // message periods in ms
    } RuntimeParameters_t;

    /**
      @brief Will acces ECU hardware via serial tty and start ros node
     **/
    ECU();
    ~ECU();
    ECU(const ECU &) = delete;
    ECU(ECU &&) = delete;
    ECU &operator=(const ECU &) = delete;
    ECU &operator=(ECU &&) = delete;

    /**
     @brief Sets PWM dutycycles for left and right motor.
            Caution: Automatically disables RPM controller.
     @param left Dutycycle of left motor from -100% to 100%
     @param right Dutycycle of right motor from -100% to 100%
     **/
    void setDutycycle(short left, short right);

    /**
     @brief Sets revolutions per minute for left and right motor.
             Caution: Automatically enables RPM controller.
     @param left Speed of left motor in RPM
     @param right Speed of right motor in RPM
     **/
    void setRPM(short left, short right);

    /**
     @brief Enables or disables RPM controller
     @param enable Enabling state
     **/
    void enableCtrlRPM(bool enable);

    /**
     @brief Turns LED on and off
     @param on LED state
     **/
    void setLED(bool on);

    /**
     @brief Trigger shitting of one ball
     **/
    void shitBall();

    /**
     @brief Requests realtime motor data measurement
     @param left samples on left motor
     @param right samples on right motor
     @param torque true for torque data, false for RPM data
     **/
    void requestRPM(uint16_t left, uint16_t right, bool torque = false);

    /**
     @brief Sets twist command for robot motion control
     @param msg twist message (linear.x and angular.z are evaluated)
     **/
    void setTwist(const geometry_msgs::msg::Twist &msg);

    /**
     @brief Change parameters of left RPM controller.
     @param Ka precontrol gain
     @param Kp proportional coefficient
     @param Tn reset time
     @param Td damping time
     **/
    void setPILeft(float Ka, float Kp, float Tn, float Td);

    /**
     @brief Change parameters of left RPM controller.
     @param Ka precontrol gain
     @param Kp proportional coefficient
     @param Tn reset time
     @param Td damping time
     **/
    void setPIRight(float Ka, float Kp, float Tn, float Td);

    /**
     @brief Set RPM setpoint lowpass filter coefficient for both motors.
     @param coeff setpoint = (1 - coeff) * setpoint_old + coeff * setpoint
     **/
    void setRPMLowpass(float coeff);

    /**
     @brief Set encoder level time correction coefficients for left encoder.
     @param corrLong multiplication factor for long level
     @param corrShort multiplication factor for short level
     **/
    void setCorrLeft(float corrLong, float corrShort);

    /**
     @brief Set encoder level time correction coefficients for right encoder.
     @param corrLong multiplication factor for long level
     @param corrShort multiplication factor for short level
     **/
    void setCorrRight(float corrLong, float corrShort);

    /**
     @brief Sets communication mode with main computer.
     @param mode 0 = ECU protocol, 1 = UART1 forwarding, 2 = UART2 forwarding, 3 = USB virtual
     commport forwarding
     */
    void setTerminalMode(uint16_t mode);

    /**
     @brief Requests realtime heading data measurement
     @param count number of samples
     **/
    void requestHeading(uint32_t count);

    /**
     @brief Performs ECU soft reset (deletes Encoder data and reconfigures runtime parameters from
     ini file)
     **/
    void reset();

    /**
     @brief Check if emergency stop button is triggered
     @return true if pressed
     **/
    bool isEmergencyStop() { return emergencyStop; }

    /**
     @brief Get latest RPM record data
     @return measurement data
     **/
    ECU::RPMData_t getRPMRecord();

    /**
     @brief Get latest Torque record data
     @return measurement data (12-bit ADC)
     **/
    ECU::RPMData_t getTorqueRecord();

    /**
     @brief Get latest heading record data
     @return measurement data in degrees
     **/
    ECU::HeadingData_t getHeadingRecord();

    /**
     @brief Get latest GPS data
     @return GPS position
     **/
    ECU::GPSData_t getGPSData() { return gpsData; }
    /**
     @brief Get latest heading data
     @return heading from -180° to 180°
     **/
    int getKVHHeading() { return kvhHeading; }

    /**
     @brief Get latest odometry data
     @return odometry position and speed
     **/
    ECU::OdometryData_t getOdometry() { return odometryData; }

    /**
     @brief Get runtime parameters
     @return current parameters
     **/
    ECU::RuntimeParameters_t getRuntimeParameters(bool readFromIni = false);

    /**
     @brief Update all runtime parameters at once
     @param params runtime data
     @param writeToIni update .ini for storing parameters
     **/
    void setRuntimeParameters(ECU::RuntimeParameters_t params, bool writeToIni = false);

  private:
    /**
     @brief Container for interface messages. Holds instruction byte, messages and CS Flag.
     **/
    typedef struct InterfaceData_t
    {
        uint8_t instruction; // Instruction byte
        uint16_t val1, val2; // Payload data
        bool csOK;           // Result of crc check
    } InterfaceData_t;

    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pubRPMRecord;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pubTorqueRecord;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pubHeadingRecord;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pubEmergencyStop;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pubKVH;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGPS;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
    rclcpp::Publisher<lois_ecu::msg::RuntimeParameters>::SharedPtr pubRuntimeParameters;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subEnableCtrlRPM;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subRequestRPM;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subRequestTorque;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subRequestHeading;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subShit;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subRequestParams;
    rclcpp::Subscription<lois_ecu::msg::RuntimeParameters>::SharedPtr subSetParams;

    bool torque;
    RPMData_t rpmRecord;
    HeadingData_t headingRecord;

    OdometryData_t odometryData;

    bool emergencyStop;

    std::thread thread;
    bool running;
    std::mutex mu;
    void worker();

    std::string port;
    int baud;

    int fd;
    struct termios term;

    float wheelWidth, wheelRadius;
    uint32_t encoderSteps;
    bool isCtrlRPM;

    std::queue<InterfaceData_t> messages;

    GPSData_t gpsData;
    int kvhHeading;

    RuntimeParameters_t runtimeParameters;
    std::string runtimeParametersIni;

    std::string odomParent, odomChild;

    /**
     @brief Sends two 16-bit integers to ECU
     @param instruction 8-bit instruction header
     @param val1 first 16-bit integer
     @param val2 secont 16-bit integer
     **/
    void send(uint8_t instruction, uint16_t val1, uint16_t val2);

    /**
     @brief Sends one 32 bit integer to ECU
     @param instruction 8-bit instruction header
     @param val 32-bit integer
     **/
    void send(uint8_t instruction, uint32_t val);

    /**
     @brief Sends one 32 bit float to ECU
     @param instruction 8-bit instruction header
     @param val 32-bit float
     **/
    void send(uint8_t instruction, float val);

    /**
     @brief Converts runtime parameters to string in .ini formag
     **/
    std::string serializeRuntimeParameters();

    /**
     @brief Reads runtime parameters from predefined .ini path
     **/
    void readRuntimeParametersFromIni();

    /**
     @brief Writes runtime parameters to predefined .ini path
     **/
    void writeRuntimeParametersToIni();

    /**
     @brief Called when a new byte is received from ECU
     **/
    void receive();

    /**
     @brief Called whenn a message from ECU is completely received.
     **/
    void processMessages();

    /**
     @brief Add rpm measurement message to buffer
     **/
    void addRPMMeasurement(InterfaceData_t *dat);

    /**
     @brief Add heading measurement message to buffer
     **/
    void addHeadingMeasurement(InterfaceData_t *dat);

    /**
     @brief Update GPS data from message
     **/
    void addGPSData(InterfaceData_t *dat);

    /**
     @brief Update odometry data from message
     **/
    void addOdometryData(InterfaceData_t *dat);
    void addOdometryDataECU(InterfaceData_t *dat);

    /**
     @brief Helper for checksum calculation
     **/
    int calculateCRC(char *message);

    /**
     @brief Helper for converting message raw data to 32-bit integer
    **/
    uint32_t uint32_2x16(uint16_t val1, uint16_t val2);

    /**
     @brief Helper for converting message raw data to 32-bit float
    **/
    float float_2x16(uint16_t val1, uint16_t val2);

    void publishRuntimeParameters();
    // Callbacks for publishing
    void callback_EmergencyStop(bool isPressed);
    void callback_RPMRecord(ECU::RPMData_t record);
    void callback_TorqueRecord(ECU::RPMData_t record);
    void callback_HeadingRecord(ECU::HeadingData_t heading);
    void callback_Heading(int heading);
    void callback_GPSData(ECU::GPSData_t position);
    void callback_Odometry(ECU::OdometryData_t position);
};
#endif
