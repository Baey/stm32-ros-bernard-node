// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <bernard_micro_ros/stm32_node.hpp>
#include <bernard_system.hpp>

BernardSystem::BernardSystem() : sensors(), gui(), status(), node() {
  // Initialize the IMU
  TwoWire *imu_i2c = new TwoWire(IMU_SDA, IMU_SCL);
  imu_i2c->begin();
  Adafruit_BNO055 *bno = new Adafruit_BNO055(55, 0x28, imu_i2c);
  status = new BernardStatus_t();
  // Create the BernardSensors object
  sensors = new BernardSensors(bno, status, gui, L_FOOT_ANALOG_PRESSURE_SENSOR,
                               R_FOOT_ANALOG_PRESSURE_SENSOR);

  // Initialize TFT display
  Adafruit_ST7735 *tft =
      new Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
  HardwareTimer *screenRefreshTimer = new HardwareTimer(TIM3);


  // Create the BernardGUI object
  gui = new BernardGUI(tft, screenRefreshTimer, status);

  // Create the STM32Node object
  node = new STM32Node(*sensors, *gui, *status);
}

void BernardSystem::init() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.print("Starting BERNARD system...");

  Serial.println("Initializing GUI...");
  // Initialize the GUI
  gui->initGUI();

  Serial.println("Initializing IMU...");
  // Initialize the IMU
  // status->IMUOnline = sensors->initIMU();

  if (status->IMUStatus == IMU_ONLINE) {
    Serial.println("IMU is online.");
  } else {
    Serial.println("IMU is offline.");
  }

  Serial.println("Initializing micro-ROS...");
  // Initialize micro-ROS
  // set_microros_serial_transports(Serial);
  status->ROSStatus = WAITING_AGENT;

  Serial.println("BERNARD system initialized.");
}
