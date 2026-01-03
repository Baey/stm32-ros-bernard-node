// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Arduino.h>
#include <gui.hpp>

/// Construct a new BernardGUI object.
BernardGUI::BernardGUI(Adafruit_ST7735 *tft, HardwareTimer *screenRefreshTimer,
                       BernardStatus_t *robotStatus, screenType_t screen)
    : tft(tft), screenRefreshTimer(screenRefreshTimer),
      currentRobotStatus(robotStatus), previousRobotStatus(), logIndex(0), previousLogIndex(0), nextScreen(screen) {

  // Initialize the log messages array
  for (int i = 0; i < 11; i++) {
    logs[i] = "";
  }
}

/// @brief Initialize the GUI and start the screen refresh timer.
void BernardGUI::initGUI() {
  currentScreen = GUI_LOG;
  nextScreen = GUI_LOG;
  previousRobotStatus = new BernardStatus_t();
  tft->initR(INITR_BLACKTAB);
  tft->fillScreen(ST77XX_BLACK);
  tft->setRotation(3);
  tft->setAddrWindow(0, 0, this->tft->width(), this->tft->height());
  tft->setTextColor(ST77XX_WHITE);
  tft->setTextSize(1);

  screenRefreshTimer->setOverflow(1, HERTZ_FORMAT);
  screenRefreshTimer->attachInterrupt(std::bind(&BernardGUI::drawScreen, this));
  screenRefreshTimer->resume();
  logMessage("Bernard GUI initialized!");
}

/// @brief Reset the screen to its default state.
void BernardGUI::resetScreen() {
  tft->fillScreen(ST77XX_BLACK);
  tft->setTextColor(ST77XX_WHITE);
  tft->setTextSize(1);
}

/// @brief Refresh the screen by checking if a screen change is requested.
void BernardGUI::drawScreen() {
  if (currentScreen != nextScreen) {
    resetScreen();
    currentScreen = nextScreen;
    return;
  }
  switch (currentScreen) {
  case GUI_STATUS:
    drawStatusScreen();
    break;
  case GUI_LOG:
    drawLogScreen();
    break;
  }
}

/// @brief Draw the status screen, displaying IMU and ROS statuses.
void BernardGUI::drawStatusScreen() {
  // Define the screen update area based on the status change
  if (previousRobotStatus != currentRobotStatus) {
    if ((previousRobotStatus->IMUStatus != currentRobotStatus->IMUStatus) ||
        (previousRobotStatus->IMUSystemStatus != currentRobotStatus->IMUSystemStatus) ||
        (previousRobotStatus->ROSStatus != currentRobotStatus->ROSStatus)) {

      // Clear the previous status area
      tft->fillRect(STATUS_VALUE_X, TOP_MARGIN, tft->width() - LEFT_MARGIN, TOP_MARGIN + LINE_HEIGHT * 2, ST7735_BLACK);

      // Draw the Kria KV260 micro-ROS agent status
      tft->setCursor(LEFT_MARGIN, TOP_MARGIN);
      tft->setTextColor(ST77XX_WHITE);
      tft->print("KRIA: ");
      tft->setCursor(STATUS_VALUE_X, TOP_MARGIN);
      switch (this->currentRobotStatus->ROSStatus) {
      case WAITING_AGENT:
        tft->setTextColor(ST77XX_YELLOW);
        tft->print("WAITING_AGENT");
        break;
      case AGENT_AVAILABLE:
        tft->setTextColor(ST77XX_GREEN);
        tft->print("AGENT_AVAILABLE");
        break;
      case AGENT_CONNECTED:
        tft->setTextColor(ST77XX_GREEN);
        tft->print("AGENT_CONNECTED");
        break;
      case AGENT_DISCONNECTED:
        tft->setTextColor(ST77XX_RED);
        tft->print("AGENT_DISCONNECTED");
        break;
      }
      tft->setTextColor(ST77XX_WHITE);

      // Draw the IMU connection status
      tft->setCursor(LEFT_MARGIN, TOP_MARGIN + LINE_HEIGHT);
      tft->print("IMU CONN: ");
      tft->setCursor(STATUS_VALUE_X, TOP_MARGIN + LINE_HEIGHT);
      if (this->currentRobotStatus->IMUStatus == IMU_ONLINE) {
        tft->setTextColor(ST77XX_GREEN);
        tft->print("ONLINE");
      } else {
        tft->setTextColor(ST77XX_RED);
        tft->print("OFFLINE");
      }
      tft->setCursor(LEFT_MARGIN, TOP_MARGIN + LINE_HEIGHT);
      tft->setTextColor(ST77XX_WHITE);

      // Draw the IMU system status
      tft->setCursor(LEFT_MARGIN, TOP_MARGIN + LINE_HEIGHT * 2);
      tft->print("IMU SYS: ");
      tft->setCursor(STATUS_VALUE_X, TOP_MARGIN + LINE_HEIGHT * 2);
      switch (this->currentRobotStatus->IMUSystemStatus) {
      case IMU_SYS_IDLE:
        tft->setTextColor(ST77XX_YELLOW);
        tft->print("IDLE");
        break;
      case IMU_SYS_ERROR:
        tft->setTextColor(ST77XX_RED);
        tft->print("ERROR");
        break;
      case IMU_SYS_INITIALIZING_PERIPHERALS:
        tft->setTextColor(ST77XX_YELLOW);
        tft->print("INIT_PERIPH");
        break;
      case IMU_SYS_INITIALIZATION:
        tft->setTextColor(ST77XX_YELLOW);
        tft->print("INIT");
        break;
      case IMU_SYS_SELF_TEST:
        tft->setTextColor(ST77XX_YELLOW);
        tft->print("SELF_TST");
        break;
      case IMU_SYS_FUSION_RUNNING:
        tft->setTextColor(ST77XX_GREEN);
        tft->print("FUSION");
        break;
      case IMU_SYS_RUNNING_NO_FUSION:
        tft->setTextColor(ST77XX_GREEN);
        tft->print("NO_FUSION");
        break;
      }
    }
    if (previousRobotStatus->euler != currentRobotStatus->euler) {
      // Clear the previous IMU euler angles area
      tft->fillRect(STATUS_VALUE_X, TOP_MARGIN + LEFT_MARGIN * 3, tft->width() - LEFT_MARGIN, LINE_HEIGHT * 6, ST7735_BLACK);

      // Draw the IMU euler angles
      tft->setTextColor(ST77XX_WHITE);
      tft->setCursor(LEFT_MARGIN, TOP_MARGIN + LINE_HEIGHT * 3);
      tft->print("EULER: ");
      tft->setCursor(STATUS_VALUE_X, TOP_MARGIN + LINE_HEIGHT * 3);
      tft->print("X: ");
      tft->setTextColor(ST77XX_RED);
      tft->print(this->currentRobotStatus->euler.x());
      tft->setCursor(STATUS_VALUE_X, TOP_MARGIN + LINE_HEIGHT * 4);
      tft->setTextColor(ST77XX_WHITE);
      tft->print("Y: ");
      tft->setTextColor(ST77XX_GREEN);
      tft->print(this->currentRobotStatus->euler.y());
      tft->setCursor(STATUS_VALUE_X, TOP_MARGIN + LINE_HEIGHT * 5);
      tft->setTextColor(ST77XX_WHITE);
      tft->print("Z: ");
      tft->setTextColor(ST77XX_BLUE);
      tft->print(this->currentRobotStatus->euler.z());
    }
  }
  *previousRobotStatus = *currentRobotStatus;
}

void BernardGUI::drawLogScreen() {
  // Clear the previous log area
  tft->fillRect(LEFT_MARGIN, TOP_MARGIN + LINE_HEIGHT * previousLogIndex, tft->width() - LEFT_MARGIN,
                TOP_MARGIN + LINE_HEIGHT * (logIndex + 1), ST7735_BLACK);
  // Draw the log messages
  for (uint8_t i = previousLogIndex; i < logIndex; i++) {
    tft->setCursor(LEFT_MARGIN, TOP_MARGIN + LINE_HEIGHT * i);
    tft->print(logs[i].c_str());
  }
  // Update the previous log index
  previousLogIndex = logIndex;
}

void BernardGUI::logMessage(const String &message) {
    logs[logIndex] = message;
    logIndex = (logIndex + 1) % 11;
}
