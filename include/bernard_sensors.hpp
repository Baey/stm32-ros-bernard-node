// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _BERNARD_SENSORS_HPP
#define _BERNARD_SENSORS_HPP

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

#include <bernard_types.hpp>
#include <gui.hpp>

/// @brief Class with BERNARD sensors.
/// @details This class so far contains the IMU sensor and the foot contact
/// sensors.
class BernardSensors
{
public:
  BernardSensors(Adafruit_BNO055 *imu, BernardStatus_t *status, BernardGUI *gui, uint32_t lFootContactPin,
                 uint32_t rFootContactPin);

  ~BernardSensors() {}

  /// @brief Initializes the Bernard sensors.
  /// @return true if the IMU sensor is online.
  IMUStatus_t initSensors();

  /// @brief Sets the IMU sensor.
  /// @details This function is used to set the IMU sensor.
  IMUStatus_t initIMU();

  /// @brief Reads the IMU quaternion.
  imu::Quaternion readQuaternion();

  /// @brief Reads the IMU linear acceleration in m/s^2.
  /// @details The IMU linear acceleration is the acceleration of the robot in
  /// the x, y and z axes.
  imu::Vector<3> readLinearAcceleration();

  /// @brief Reads the IMU accelerometer data in m/s^2.
  /// @details The IMU accelerometer data is the acceleration of the robot in
  /// the x, y and z axes.
  imu::Vector<3> readAccelerometer();

  /// @brief Reads IMU data in batch (optimized for reduced I2C transactions).
  /// @details Reads quaternion, linear acceleration, and gyroscope in one I2C operation
  /// to reduce latency compared to separate reads.
  void readIMUDataBatch(imu::Quaternion &quat, imu::Vector<3> &linearAcc, imu::Vector<3> &gyro);

  /// @brief Reads the IMU gyroscope data in rad/s.
  /// @details The IMU gyroscope data is the angular velocity of the robot in
  /// the x, y and z axes.
  imu::Vector<3> readGyroscope();

  /// @brief Reads the IMU temperature data in °C.
  /// @return The IMU temperature data.
  int8_t readTemperature();

  /// @brief Reads analog values from the foot contact sensors.
  /// @details The foot contact sensors are analog sensors that return a value
  /// between 0 and 1023.
  /// @details The foot contact sensors are connected to the analog pins of the
  /// microcontroller.
  /// @details The foot contact sensors are used to detect if the robot is in
  /// contact with the ground.
  /// @return An array of uint16_t containing the analog values from the left
  /// and right foot contact sensors.
  std::array<uint16_t, 2> readFootPressure();

  /// @brief Gets the IMU quaternion.
  /// @return The IMU quaternion.
  imu::Quaternion getQuaternion() const { return quat; }

  /// @brief Gets the IMU linear acceleration.
  /// @return The IMU linear acceleration.
  imu::Vector<3> getLinearAcceleration() const { return linearAcc; }

  /// @brief Gets the IMU accelerometer data.
  /// @return The IMU accelerometer data.
  imu::Vector<3> getAccelerometer() const { return angularAcc; }

  /// @brief Gets the IMU gyroscope data.
  /// @return The IMU gyroscope data.
  imu::Vector<3> getGyroscope() const { return gyro; }

  /// @brief Gets the IMU temperature data.
  /// @return The IMU temperature data.
  int8_t getTemperature() const { return temp; }

  /// @brief Gets the foot contact sensors values.
  /// @return A vector of uint16_t containing the analog values from the left
  /// and right foot contact sensors.
  const std::array<uint16_t, 2>& getFootPressure() const
  {
    return footContactValues;
  }

  /// @brief Timer callback function to read the IMU data and the foot contact sensors.
  /// @details This function is called every 100ms to read the IMU data and the
  /// foot contact sensors.
  void readHighFrequencyTimerCallback();

  /// @brief Timer callback function to read the IMU temperature.
  void readLowFrequencyTimerCallback();

  /// @brief Timer callback function to ping the IMU sensor.
  void imuStatusTimerCallback();

private:
  Adafruit_BNO055 *imu;
  BernardGUI *gui;
  sensors_event_t imuEvent;
  BernardStatus_t *status;
  uint8_t imuSystemStatus, imuSelfTestResult, imuSystemError;
  HardwareTimer *highFreqTimer, *lowFreqTimer, *pingImuTimer;
  uint32_t lFootContactPin;
  uint32_t rFootContactPin;
  imu::Quaternion quat;
  imu::Vector<3> linearAcc;
  imu::Vector<3> angularAcc;
  imu::Vector<3> gyro;
  int8_t temp;
  std::array<uint16_t, 2> footContactValues;
};

#endif // _BERNARD_SENSORS_HPP
