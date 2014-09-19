#ifndef RC_MSG_SRV_H
#define RC_MSG_SRV_H

#include <stdlib.h>
#include <string>
#include <vector>

#include "std_msgs/Header.h"

using namespace std;

namespace robocontroller
{

/**
 * @enum MotorCtrlMode
 * @brief Motor control modes.
 */
typedef enum
{
    mcDirectPWM = 0,    /**< PID is not active, speeds are in PWM mode: 0 -> Motor Stopped, +100/-100 -> Motor at max rate */
    mcPID = 1           /**< PID is active, speeds are in mm/sec */
} MotorCtrlMode;

/**
 * @enum MotorCtrlMode
 * @brief Motor control modes.
 */
typedef enum
{
    motLeft = 0,    /**< Used to indicate the left motor */
    motRight = 1    /**< Used to indicate the right motor */
} MotorPos;

/**
 * @enum Option
 * @brief ON/OFF for toggle options
 */
typedef enum
{
    Off = 0,    /**< Option not active */
    On = 1      /**< Option active */
} Option;

/**
 * @enum PinLevel
 * @brief High/Low for pin levels
 */
typedef enum
{
    Low = 0, /**< Low Active */
    High = 1 /**< High Active */
} PinLevel;

/**
 * @enum EncoderPos
 * @brief Indicates the position of the Encoder
 */
typedef enum
{
    Motor = 0,  /**< Encoder suited on the shaft of the motor */
    Wheel = 1   /**< Encoder suited on the shaft of the wheel */
} EncoderPos;

/**
 * @enum AnalogCalibValue
 * @brief Indicates the type of value for Analogic Ports calibration
 */
typedef enum
{
    CalLow = 0, /**< Lower Value for analogic calibration */
    CalHigh = 1 /**< Higher Value for analogic calibration */
} AnalogCalibValue;

/**
  * @struct _BoardStatus
  * @brief Used to mantain the state of the
  *        configuration of the board
  */
typedef struct _BoardStatus
{
    bool pidEnable;         /**< Indicates if the motor PID controls are enabled */
    bool wdEnable;          /**< Indicates if the Command WatchDod is enabled. If it is true and the board does not receive command for N msec (@ref getWatchDogTime and @ref setWatchDogTime)the motors stop */
    bool saveToEeprom;      /**< Indicates if the Board parameters are saved to Eeprom when changed */
    bool accelRampEnable;   /**< Indicates if acceleration is limited by speed ramps */
} BoardStatus;

/**
  * @struct _RobotConfiguration
  * @brief Used to mantain the state of the configuration of the robot
  */
typedef struct _RobotConfiguration
{
    // >>>>> Dimensions
    int Weight; /**< Robot Weight (g) */
    int Width;  /**< Robot Width (mm) */
    int Height; /**< Robot Height (mm) */
    int Lenght; /**< Robot Lenght (mm) */
    // <<<<< Dimensions

    // >>>>> Wheels, Motors and Reduction
    int WheelBase;              /**< Distance between the center of the Wheels (mm) */
    int WheelRadiusLeft;        /**< Radius of the left wheel (0.01mm) */
    int WheelRadiusRight;       /**< Radius of the right wheel (0.01mm) */
    int EncoderCprLeft;         /**< Count per Round of the left encoder */
    int EncoderCprRight;        /**< Count per Round of the right encoder */
    int MaxRpmMotorLeft;        /**< Max RPM of the left motor */
    int MaxRpmMotorRight;       /**< Max RPM of the right motor */
    int MaxAmpereMotorLeft;     /**< Max current assorbed by left motor (mA) */
    int MaxAmpereMotorRight;    /**< Max current assorbed by right motor (mA) */
    int MaxTorqueMotorLeft;     /**< Max torque of the left motor (Ncm) */
    int MaxTorqueMotorRight;    /**< Max torque of the right motor (Ncm) */
    int RatioShaftLeft;         /**< Reduction Ratio from the shaft of the left motor to the shaft of the left wheel*/
    int RatioShaftRight;        /**< Reduction Ratio from the shaft of the right motor to the shaft of the right wheel*/
    int RatioMotorLeft;         /**< Reduction Ratio on the left Motor Shaft (Put 1 if you set Max RPM considering it just reduced) */
    int RatioMotorRight;        /**< Reduction Ratio on the right Motor Shaft (Put 1 if you set Max RPM considering it just reduced) */
    PinLevel MotorEnableLevel;      /**< Enable Level of the Robot Driver (Low/High)*/
    EncoderPos EncoderPosition;     /**< Encoder on the shaft of the motor or of the wheel */
    // <<<<< Wheels, Motors and Reduction

    // >>>>> Battery
    int MaxChargedBatteryLevel; /**< Value of the power battery fully charged (Volts * 1000) */
    int MinChargedBatteryLevel; /**< Value of the power battery to be considered discharged (Volts * 1000) */
    // <<<<< Battery
} RobotConfiguration;

/**
  * @struct _RobotTelemetry
  * @brief Used to keep track of the telemetry of the robot
  */
typedef struct _RobotTelemetry
{
    int16_t PwmLeft;         /**< Last value of the PWM for left wheel */
    int16_t PwmRight;        /**< Last value of the PWM for right wheel */
    double RpmLeft;          /**< Last RPM for left wheel */
    double RpmRight;         /**< Last RPM for right wheel */
    double LinSpeedLeft;     /**< Last Linear Speed for left Wheel */
    double LinSpeedRight;    /**< Last Linear Speed for right Wheel */
    double Battery;          /**< Last battery voltage */
} RobotTelemetry;

/**
  * @struct _RobotPose
  * @brief Used to keep track of the pose of the robot over the time
  */
typedef struct _RobotPose
{
    double x;                   /**< Coordinate X */
    double y;                   /**< Coordinate Y */
    double theta;               /**< Orientation */
} RobotPose;

/**
  * @struct _RcDebug
  * @brief Used to get information of RoboController register state
  */
typedef struct _RcDebug
{
    u_int16_t enc1_period;      /**< WORD_ENC1_PERIOD */
    u_int16_t enc2_period;      /**< WORD_ENC2_PERIOD */
} RcDebug;

}

#endif // RC_MSG_SRV_H
