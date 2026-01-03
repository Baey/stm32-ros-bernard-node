// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_Sensor.h>
#include <micro_ros_platformio.h>
#include <utility/imumaths.h>

#include "bernard_micro_ros/stm32_node.hpp"
#include "bernard_types.hpp"
#include "bernard_wiring.hpp"
#include "gui.hpp"

TwoWire imu_i2c(IMU_SDA, IMU_SCL);
// HardwareTimer *imuTimer = new HardwareTimer(TIM2);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &imu_i2c);
// imu::Quaternion quat;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
HardwareTimer screenRefreshTimer(TIM2);
BernardStatus_t bernardStatus;

BernardGUI gui(&tft, &screenRefreshTimer, &bernardStatus);
BernardSensors sensors(&bno, &bernardStatus, &gui, L_FOOT_ANALOG_PRESSURE_SENSOR,
                       R_FOOT_ANALOG_PRESSURE_SENSOR);
STM32Node node(sensors, gui);

void setup(void) {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(921600);

    // Configure I2C bus for IMU at 400 kHz (Fast Mode) to reduce read latency
    imu_i2c.begin();
    imu_i2c.setClock(400000);  // 400 kHz instead of default 100 kHz

    gui.initGUI();
    delay(1000);

    gui.logMessage("ROS initialization...");
    delay(1000);
    set_microros_serial_transports(Serial);
    bernardStatus.ROSStatus = WAITING_AGENT;

    bernardStatus.IMUStatus = sensors.initSensors();
    gui.logMessage("Sensors initialized!");
    delay(1000);
    gui.logMessage("Waiting for Kria...");
    delay(1000);
}

void loop(void) {
    switch (bernardStatus.ROSStatus) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(1000, bernardStatus.ROSStatus =
                                         (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                             ? AGENT_AVAILABLE
                                             : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            gui.logMessage("Kria agent available. Connecting...");
			if (node.createEntities()) {
				bernardStatus.ROSStatus = AGENT_CONNECTED;
				digitalWrite(LED_BUILTIN, 1);
				gui.setNextScreen(GUI_STATUS);
			} else {
				bernardStatus.ROSStatus = WAITING_AGENT;
				digitalWrite(LED_BUILTIN, 0);
				node.destroyEntities();
				gui.logMessage("Failed to connect to Kria agent.");
			}
            break;
        case AGENT_CONNECTED:
            if (bernardStatus.ROSStatus == AGENT_CONNECTED) {
                node.spin();
            }
            break;
        case AGENT_DISCONNECTED:
            node.destroyEntities();
            bernardStatus.ROSStatus = WAITING_AGENT;
			digitalWrite(LED_BUILTIN, 0);
            break;
        default:
            break;
    }
}
