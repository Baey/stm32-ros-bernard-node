// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include <Arduino.h>
#include <bernard_micro_ros/stm32_node.hpp>

// Definition of the static member.
STM32Node *STM32Node::instance = nullptr;

STM32Node::STM32Node(BernardSensors &sensors, BernardGUI &gui)
    : support(), node(), timer(), tempTimer(), executor(), allocator(),
      pubFeetPressure(), pubImu(), pubTemp(), msgFeetPressure(nullptr),
      msgImu(nullptr), msgTemp(nullptr), sensors(&sensors), gui(&gui) {}

/// @brief Destructor that destroys all created ROS entities.
STM32Node::~STM32Node() { destroyEntities(); }

/// @brief Creates all necessary ROS entities.
/// @return true on success.
bool STM32Node::createEntities()
{
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));

    RCCHECK(rclc_node_init_default(&this->node, "stm32_publisher_rclc", "",
                                   &this->support));

    // Foot contact publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &pubFeetPressure, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        "feet_pressure"));

    // IMU publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &pubImu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));

    // Temperature publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &pubTemp, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "imu/temp"));

    // Allocate message objects on heap
    msgFeetPressure = new std_msgs__msg__UInt16MultiArray();
    msgImu = new sensor_msgs__msg__Imu();
    msgTemp = new std_msgs__msg__Int8();
    gui->logMessage("Allocated message objects.");
    if (rosidl_runtime_c__uint16__Sequence__init(&msgFeetPressure->data, 2) != true) {
        delete msgFeetPressure;
        delete msgImu;
        delete msgTemp;
        return false;
    }
    msgFeetPressure->data.size = 2;
    msgFeetPressure->data.capacity = 2;

    // Fast timer (e.g., 200 Hz -> 5 ms)
    constexpr uint64_t timer_timeout_ns = 7400000;
    RCCHECK(rclc_timer_init_default(&this->timer, &this->support,
                                    timer_timeout_ns,
                                    this->timerCallback));

    // Temperature timer (e.g., 1 Hz -> 1000 ms)
    constexpr uint64_t temp_timer_timeout_ms = 1000;
    RCCHECK(rclc_timer_init_default(&this->tempTimer, &this->support,
                                    RCL_MS_TO_NS(temp_timer_timeout_ms),
                                    this->tempTimerCallback));

    STM32Node::instance = this;
    executor = rclc_executor_get_zero_initialized_executor();
    // Two timers -> 2 handles in executor
    RCCHECK(rclc_executor_init(&this->executor, &this->support.context, 2,
                               &this->allocator));
    RCCHECK(rclc_executor_add_timer(&this->executor, &this->timer));
    RCCHECK(rclc_executor_add_timer(&this->executor, &this->tempTimer));

    return true;
}

/// @brief Destroys all created ROS entities.
void STM32Node::destroyEntities()
{
    if (STM32Node::instance == nullptr) return;

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    STM32Node::instance->retFeetPressure = rcl_publisher_fini(&pubFeetPressure, &node);
    STM32Node::instance->retImu = rcl_publisher_fini(&pubImu, &node);
    STM32Node::instance->retTimer = rcl_timer_fini(&timer);
    STM32Node::instance->retTempTimer = rcl_timer_fini(&tempTimer);
    STM32Node::instance->retExecutor = rclc_executor_fini(&executor);
    STM32Node::instance->retNode = rcl_node_fini(&node);

    // Deallocate message objects
    if (msgFeetPressure != nullptr) {
        delete msgFeetPressure;
        msgFeetPressure = nullptr;  // Prevent double-delete
    }
    if (msgImu != nullptr) {
        delete msgImu;
        msgImu = nullptr;
    }
    if (msgTemp != nullptr) {
        delete msgTemp;
        msgTemp = nullptr;
    }

    rclc_support_fini(&support);
    STM32Node::instance = nullptr;
}

/// @brief Spins the executor to process available callbacks.
void STM32Node::spin() { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); }

/// @brief Timer callback that toggles the boolean messages and publishes them.
/// @param timer The timer object.
/// @param last_call_time The last call time.
void STM32Node::timerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (STM32Node::instance != nullptr)
    {
        STM32Node *node = STM32Node::instance;

        const std::array<uint16_t, 2>& footPressure = node->sensors->getFootPressure();
        const imu::Quaternion quat = node->sensors->getQuaternion();
        const imu::Vector<3> linearAcc = node->sensors->getLinearAcceleration();
        const imu::Vector<3> gyro = node->sensors->getGyroscope();

        node->msgFeetPressure->data.data[0] = footPressure[0];
        node->msgFeetPressure->data.data[1] = footPressure[1];

        uint32_t timestamp = millis();
        node->msgImu->header.stamp.sec = timestamp / 1000;
        node->msgImu->header.stamp.nanosec = (timestamp % 1000) * 1000000;
        node->msgImu->orientation.x = quat.x();
        node->msgImu->orientation.y = quat.y();
        node->msgImu->orientation.z = quat.z();
        node->msgImu->orientation.w = quat.w();
        node->msgImu->linear_acceleration.x = linearAcc.x();
        node->msgImu->linear_acceleration.y = linearAcc.y();
        node->msgImu->linear_acceleration.z = linearAcc.z();
        node->msgImu->angular_velocity.x = gyro.x();
        node->msgImu->angular_velocity.y = gyro.y();
        node->msgImu->angular_velocity.z = gyro.z();

        node->retFeetPressure = rcl_publish(&STM32Node::instance->pubFeetPressure,
                                            STM32Node::instance->msgFeetPressure, NULL);
        if (node->retFeetPressure != RCL_RET_OK) {
            const String error_message = "Failed to publish feet pressure! Error code: " + String(node->retFeetPressure);
            node->gui->logMessage(error_message);
        }
        node->retImu = rcl_publish(&STM32Node::instance->pubImu,
                                   STM32Node::instance->msgImu, NULL);
        if (node->retImu != RCL_RET_OK) {
            const String error_message = "Failed to publish IMU data! Error code: " + String(node->retImu);
            node->gui->logMessage(error_message);
        }
    }
}

void STM32Node::tempTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    (void)timer;
    if (STM32Node::instance != nullptr)
    {
        STM32Node *node = STM32Node::instance;
        node->msgTemp->data = node->sensors->getTemperature();
        node->retTemp = rcl_publish(&STM32Node::instance->pubTemp,
                                    STM32Node::instance->msgTemp, NULL);
    }
}
