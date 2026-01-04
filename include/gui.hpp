// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _GUI_HPP
#define _GUI_HPP

#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <bernard_types.hpp>
#include <bernard_wiring.hpp>

#define LEFT_MARGIN 10
#define TOP_MARGIN 10
#define LINE_HEIGHT 10
#define STATUS_VALUE_X 65

/// @brief Available screen types.
typedef enum {
    GUI_STATUS,
    GUI_LOG,
} screenType_t;

/// @brief Class for the Bernard GUI.
/// @details This class is responsible for displaying the robot status on the TFT screen.
class BernardGUI {
   public:
    /// @brief Constructor for the BernardGUI class.
    /// @param tft Pointer to the TFT display object.
    /// @param screenRefreshTimer Pointer to the hardware timer for screen refresh.
    /// @param robotStatus Pointer to the robot status object.
    /// @details This constructor initializes the GUI with the provided TFT display,
    /// hardware timer, and robot status object.
    BernardGUI(Adafruit_ST7735* tft, HardwareTimer* screenRefreshTimer,
               BernardStatus_t* robotStatus, screenType_t screen = GUI_LOG);

    /// @brief Initialize the GUI.
    void initGUI();

    /// @brief Reset the screen to its default state.
    void resetScreen();

    /// @brief Draw the screen based on the current screen type.
    void drawScreen();

    /// @brief Draw the status screen, displaying IMU and ROS statuses.
    void drawStatusScreen();

    /// @brief Draw the log screen, displaying log messages.
    void drawLogScreen();

    /// @brief Log a message to the screen.
    /// @param message The message to log.
    /// @details This function adds a message to the log and updates the display.
    void logMessage(const String& message);

    /// @brief Set the next screen to be displayed.
    /// @param screen The next screen type to be displayed.
    /// @details This function sets the next screen to be displayed and triggers a screen refresh.
    void setNextScreen(screenType_t screen) {
        nextScreen = screen;
    }

   private:
    Adafruit_ST7735* tft;
    HardwareTimer* screenRefreshTimer;
    BernardStatus_t* currentRobotStatus;
    BernardStatus_t* previousRobotStatus;
    screenType_t currentScreen, nextScreen;
    String logs[11];
    uint8_t logIndex, previousLogIndex;
    bool initialStatusDraw{true};
};

#endif  // _GUI_HPP
