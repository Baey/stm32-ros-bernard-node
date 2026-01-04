// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "bernard_micro_ros/stm32_node.hpp"

#include <Arduino.h>
#include <std_msgs/msg/string.h>

// Definition of the static member.
STM32Node* STM32Node::instance = nullptr;

STM32Node::STM32Node(BernardSensors& sensors, BernardGUI& gui, BernardStatus_t& status)
    : support(), node(), timer(), tempTimer(), executor(), allocator(), pubFeetPressure(),
    pubImu(), pubTemp(), subIp(), msgFeetPressure(nullptr), msgImu(nullptr), msgTemp(nullptr),
    sensors(&sensors), gui(&gui), status(&status) {}

STM32Node::~STM32Node() {
    destroyEntities();
}

bool STM32Node::createEntities() {
    allocator = rcl_get_default_allocator();
    errorCounter = 0;

    RCCHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));
    RCCHECK(rclc_node_init_default(&this->node, "stm32_publisher_rclc", "", &this->support));

    // Foot contact publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &pubFeetPressure, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        "feet_pressure"));

    // IMU publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &pubImu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));

    // Temperature publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &pubTemp, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "imu/temp"));

    // Allocate message objects
    msgFeetPressure = new std_msgs__msg__UInt16MultiArray();
    msgImu = new sensor_msgs__msg__Imu();
    msgTemp = new std_msgs__msg__Int8();
    msgIp = new std_msgs__msg__String();
    if (!rosidl_runtime_c__uint16__Sequence__init(&msgFeetPressure->data, 2)) {
        delete msgFeetPressure;
        delete msgImu;
        delete msgTemp;
        return false;
    }
    msgFeetPressure->data.size = 2;
    msgFeetPressure->data.capacity = 2;

    std_msgs__msg__String__init(msgIp);
    const size_t ip_capacity = 64;
    msgIp->data.data = (char*)malloc(ip_capacity * sizeof(char));
    msgIp->data.capacity = ip_capacity;
    msgIp->data.size = 0;

    // IP subscription
    RCCHECK(rclc_subscription_init_best_effort(
        &subIp, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "status/ip"));

    // Fast timer (200 Hz -> 5 ms)
    constexpr uint64_t timer_timeout_ns = 7400000;
    RCCHECK(rclc_timer_init_default(&this->timer, &this->support,
                                    timer_timeout_ns, this->timerCallback));

    // Temperature timer (1 Hz -> 1000 ms)
    constexpr uint64_t temp_timer_timeout_ms = 1000;
    RCCHECK(rclc_timer_init_default(&this->tempTimer, &this->support,
                                    RCL_MS_TO_NS(temp_timer_timeout_ms),
                                    this->tempTimerCallback));

    STM32Node::instance = this;
    executor = rclc_executor_get_zero_initialized_executor();

    // Executor with 2 timers + 1 subscription -> 3 handles
    RCCHECK(rclc_executor_init(&this->executor, &this->support.context, 3, &this->allocator));
    RCCHECK(rclc_executor_add_timer(&this->executor, &this->timer));
    RCCHECK(rclc_executor_add_timer(&this->executor, &this->tempTimer));
    RCCHECK(rclc_executor_add_subscription(
        &this->executor,
        &this->subIp,
        msgIp,
        &this->ipCallback,
        ON_NEW_DATA));

    return true;
}

void STM32Node::destroyEntities() {
    if (!instance) return;

    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    instance->retFeetPressure = rcl_publisher_fini(&pubFeetPressure, &node);
    instance->retImu = rcl_publisher_fini(&pubImu, &node);
    instance->retTemp = rcl_publisher_fini(&pubTemp, &node);
    rcl_subscription_fini(&subIp, &node);
    instance->retTimer = rcl_timer_fini(&timer);
    instance->retTempTimer = rcl_timer_fini(&tempTimer);
    instance->retExecutor = rclc_executor_fini(&executor);
    instance->retNode = rcl_node_fini(&node);

    if (msgFeetPressure) {
        delete msgFeetPressure;
        msgFeetPressure = nullptr;
    }
    if (msgImu) {
        delete msgImu;
        msgImu = nullptr;
    }
    if (msgTemp) {
        delete msgTemp;
        msgTemp = nullptr;
    }
    if (msgIp) {
        std_msgs__msg__String__fini(msgIp);
        delete msgIp;
        msgIp = nullptr;
    }

    errorCounter = 0;
    rclc_support_fini(&support);
    instance = nullptr;
}

void STM32Node::spin() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}

void STM32Node::timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;

    if (!STM32Node::instance) return;
    STM32Node* node = STM32Node::instance;

    const auto& footPressure = node->sensors->getFootPressure();
    const auto quat = node->sensors->getQuaternion();
    const auto linearAcc = node->sensors->getLinearAcceleration();
    const auto gyro = node->sensors->getGyroscope();

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

    node->retFeetPressure = rcl_publish(&node->pubFeetPressure, node->msgFeetPressure, NULL);
    node->retImu = rcl_publish(&node->pubImu, node->msgImu, NULL);

    if (node->retFeetPressure != RCL_RET_OK || node->retImu != RCL_RET_OK) {
        node->errorCounter++;
        if (node->errorCounter >= MAX_ERRORS_COUNT) {
            node->status->ROSStatus = AGENT_DISCONNECTED;
            node->gui->setNextScreen(GUI_LOG);
            node->gui->logMessage("Max ROS2 publish errors reached!");
        }
    } else {
        node->errorCounter = 0;
    }
}

void STM32Node::tempTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    (void)timer;

    if (!STM32Node::instance) return;
    STM32Node* node = STM32Node::instance;

    node->msgTemp->data = node->sensors->getTemperature();
    node->retTemp = rcl_publish(&node->pubTemp, node->msgTemp, NULL);
}

void STM32Node::ipCallback(const void* msgin) {
    const auto& msg = *static_cast<const std_msgs__msg__String*>(msgin);
    if (STM32Node::instance) {
        STM32Node* node = STM32Node::instance;
        node->status->ipAddress = msg.data.data;
    }
}