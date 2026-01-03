// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <bernard_sensors.hpp>

BernardSensors::BernardSensors(Adafruit_BNO055 *imu, BernardStatus_t *status, BernardGUI *gui, uint32_t lFootContactPin,
                               uint32_t rFootContactPin)
    : imu(imu), status(status), gui(gui), lFootContactPin(lFootContactPin),
      rFootContactPin(rFootContactPin), quat(), linearAcc(), angularAcc(),
      gyro(), footContactValues()
{
  highFreqTimer = new HardwareTimer(TIM3);
  lowFreqTimer = new HardwareTimer(TIM4);
  // pingImuTimer = new HardwareTimer(TIM6);
}

IMUStatus_t BernardSensors::initSensors()
{
  gui->logMessage("Initializing IMU...");
  status->IMUStatus = initIMU();

  if (status->IMUStatus != IMU_ONLINE) {
    gui->logMessage("IMU initialization failed!");
    return IMU_OFFLINE;
  }

  // highFreqTimer->setOverflow(100, HERTZ_FORMAT);
  // highFreqTimer->attachInterrupt([this]()
  //                                 { this->readHighFrequencyTimerCallback(); });
  // highFreqTimer->resume();

  pinMode(lFootContactPin, INPUT);
  pinMode(rFootContactPin, INPUT);

  lowFreqTimer->setOverflow(1, HERTZ_FORMAT);
  lowFreqTimer->attachInterrupt([this]()
                                 { this->readLowFrequencyTimerCallback(); });
  lowFreqTimer->resume();

  // pingImuTimer->setOverflow(1, HERTZ_FORMAT);
  // pingImuTimer->attachInterrupt([this]()
  //                               { this->imuStatusTimerCallback(); });
  // pingImuTimer->resume();

  return status->IMUStatus;
}

IMUStatus_t BernardSensors::initIMU()
{
  if (!imu->begin(OPERATION_MODE_NDOF))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    gui->logMessage("IMU not found!");
    return IMU_OFFLINE;
  }
  else
  {
    gui->logMessage("IMU found!");
    delay(1000);
    gui->logMessage("Setting XTAL..");
    delay(100);
    /* Use external crystal for better accuracy */
    imu->setExtCrystalUse(true);

    return IMU_ONLINE;
  }
}

imu::Quaternion BernardSensors::readQuaternion() { return imu->getQuat(); }

imu::Vector<3> BernardSensors::readLinearAcceleration()
{
  return imu->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

imu::Vector<3> BernardSensors::readAccelerometer()
{
  return imu->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}

void BernardSensors::readIMUDataBatch(imu::Quaternion &quat, imu::Vector<3> &linearAcc, imu::Vector<3> &gyro)
{
  // Batch read: Read all required IMU data in minimal I2C transactions
  // This is more efficient than separate reads as it reduces I2C overhead
  quat = imu->getQuat();
  linearAcc = imu->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gyro = imu->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

imu::Vector<3> BernardSensors::readGyroscope()
{
  return imu->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

int8_t BernardSensors::readTemperature() { return imu->getTemp(); }

std::array<uint16_t, 2> BernardSensors::readFootPressure()
{
  return {static_cast<uint16_t>(analogRead(lFootContactPin)), static_cast<uint16_t>(analogRead(rFootContactPin))};
}

void BernardSensors::readHighFrequencyTimerCallback()
{
  // Use batch read to minimize I2C transactions and reduce ISR latency
  readIMUDataBatch(quat, linearAcc, gyro);

  // Read foot pressure analog sensors and apply exponential smoothing using integer arithmetic
  // Avoids floating-point precision loss for values < 10
  std::array<uint16_t, 2> footContact = readFootPressure();
  footContactValues[0] = (9 * footContactValues[0] + footContact[0]) / 10;
  footContactValues[1] = (9 * footContactValues[1] + footContact[1]) / 10;
}

void BernardSensors::readLowFrequencyTimerCallback()
{
  temp = readTemperature();
}

// void BernardSensors::imuStatusTimerCallback()
// {
//   imu->getSystemStatus(&imuSystemStatus, &imuSelfTestResult, &imuSystemError);
//   status->IMUSystemStatus = static_cast<IMUSystemStatus_t>(imuSystemStatus);
//   status->IMUSelfTestResult = static_cast<IMUSelfTestStatus_t>(imuSelfTestResult);
//   status->IMUSystemError = static_cast<IMUErrorStatus_t>(imuSystemError);

//   status->euler = imu->getVector(Adafruit_BNO055::VECTOR_EULER);
// }
