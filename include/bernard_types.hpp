// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _BERNARD_TYPES_HPP
#define _BERNARD_TYPES_HPP

#include <Adafruit_BNO055.h>

/// @brief Status types reported by MicroROS.
enum microROSStatus_t {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

/// @brief IMU online status.
enum IMUStatus_t {
    IMU_OFFLINE,
    IMU_ONLINE
};

/// @brief IMU system status according to the BNO055 datasheet.
enum IMUSystemStatus_t {
    IMU_SYS_IDLE = 0,                      // 0 = Idle
    IMU_SYS_ERROR = 1,                     // 1 = System Error
    IMU_SYS_INITIALIZING_PERIPHERALS = 2,  // 2 = Initializing Peripherals
    IMU_SYS_INITIALIZATION = 3,            // 3 = System Initialization
    IMU_SYS_SELF_TEST = 4,                 // 4 = Executing Self-Test
    IMU_SYS_FUSION_RUNNING = 5,            // 5 = Sensor fusion algorithm running
    IMU_SYS_RUNNING_NO_FUSION = 6          // 6 = System running without fusion algorithms
};

/// @brief IMU self test status bits according to the BNO055 datasheet.
enum IMUSelfTestStatus_t {
    IMU_SELF_TEST_ACCELEROMETER = 0x01,  // Bit 0
    IMU_SELF_TEST_MAGNETOMETER = 0x02,   // Bit 1
    IMU_SELF_TEST_GYROSCOPE = 0x04,      // Bit 2
    IMU_SELF_TEST_MCU = 0x08,            // Bit 3
    IMU_SELF_TEST_ALL_PASSED = 0x0F      // All tests passed
};

/// @brief IMU error status according to the BNO055 datasheet section 4.3.59.
enum IMUErrorStatus_t {
    IMU_ERR_NONE = 0,                       // 0 = No error
    IMU_ERR_PERIPHERAL_INIT = 1,            // 1 = Peripheral initialization error
    IMU_ERR_SYSTEM_INIT = 2,                // 2 = System initialization error
    IMU_ERR_SELF_TEST_FAILED = 3,           // 3 = Self test result failed
    IMU_ERR_REG_MAP_VAL_OUT_OF_RANGE = 4,   // 4 = Register map value out of range
    IMU_ERR_REG_MAP_ADDR_OUT_OF_RANGE = 5,  // 5 = Register map address out of range
    IMU_ERR_REG_MAP_WRITE = 6,              // 6 = Register map write error
    IMU_ERR_LOW_POWER_UNAVAILABLE = 7,      // 7 = BNO low power mode not available for selected operation mode
    IMU_ERR_ACCEL_POWER_UNAVAILABLE = 8,    // 8 = Accelerometer power mode not available
    IMU_ERR_FUSION_CONFIG = 9,              // 9 = Fusion algorithm configuration error
    IMU_ERR_SENSOR_CONFIG = 0xA             // A = Sensor configuration error
};

/// @brief Datatype representing the robot status.
typedef struct {
    IMUStatus_t IMUStatus;
    IMUSystemStatus_t IMUSystemStatus;
    IMUSelfTestStatus_t IMUSelfTestResult;
    IMUErrorStatus_t IMUSystemError;
    microROSStatus_t ROSStatus;
    imu::Vector<3> euler;
    int8_t temperature;
    String ipAddress = "NO_IP";
} BernardStatus_t;

/// @brief comparator for BernardStatus_t.
/// @details This function is used to compare two BernardStatus_t objects.
/// @param lhs The left-hand side object.
/// @param rhs The right-hand side object.
/// @return true if the objects are equal, false otherwise.
inline bool operator==(const BernardStatus_t& lhs, const BernardStatus_t& rhs) {
    return (lhs.IMUStatus == rhs.IMUStatus &&
            lhs.IMUSystemStatus == rhs.IMUSystemStatus &&
            lhs.IMUSystemError == rhs.IMUSystemError &&
            lhs.ROSStatus == rhs.ROSStatus);
}

/// @brief comparator for BernardStatus_t.
/// @details This function is used to compare two BernardStatus_t objects.
/// @param lhs The left-hand side object.
/// @param rhs The right-hand side object.
/// @return true if the objects are not equal, false otherwise.
inline bool operator!=(const BernardStatus_t& lhs, const BernardStatus_t& rhs) {
    return !(lhs == rhs);
}

/// @brief comparator for imu::Vector<3>.
/// @details This function is used to compare two imu::Vector<3> objects.
/// @param lhs The left-hand side object.
/// @param rhs The right-hand side object.
/// @return true if the objects are equal, false otherwise.
inline bool operator==(const imu::Vector<3>& lhs, const imu::Vector<3>& rhs) {
    return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
}

/// @brief comparator for imu::Vector<3>.
/// @details This function is used to compare two imu::Vector<3> objects.
/// @param lhs The left-hand side object.
/// @param rhs The right-hand side object.
/// @return true if the objects are not equal, false otherwise.
inline bool operator!=(const imu::Vector<3>& lhs, const imu::Vector<3>& rhs) {
    return !(lhs == rhs);
}

#endif  // _BERNARD_TYPES_HPP
