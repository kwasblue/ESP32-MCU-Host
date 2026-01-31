// include/config/FeatureFlags.h
// Central feature flag definitions with dependency validation
// All optional features can be enabled/disabled via platformio.ini build_flags
#pragma once

// =============================================================================
// TRANSPORT FLAGS
// =============================================================================
#ifndef HAS_WIFI
#define HAS_WIFI 0
#endif

#ifndef HAS_BLE
#define HAS_BLE 0
#endif

#ifndef HAS_UART_TRANSPORT
#define HAS_UART_TRANSPORT 1  // Default: always have UART
#endif

#ifndef HAS_MQTT_TRANSPORT
#define HAS_MQTT_TRANSPORT 0
#endif

// MQTT requires WiFi
#if HAS_MQTT_TRANSPORT && !HAS_WIFI
#error "HAS_MQTT_TRANSPORT requires HAS_WIFI=1"
#endif

// =============================================================================
// MOTOR FLAGS
// =============================================================================
#ifndef HAS_DC_MOTOR
#define HAS_DC_MOTOR 0
#endif

#ifndef HAS_SERVO
#define HAS_SERVO 0
#endif

#ifndef HAS_STEPPER
#define HAS_STEPPER 0
#endif

#ifndef HAS_ENCODER
#define HAS_ENCODER 0
#endif

#ifndef HAS_DC_VELOCITY_PID
#define HAS_DC_VELOCITY_PID 0
#endif

// DC velocity PID requires both DC motor and encoder
#if HAS_DC_VELOCITY_PID && (!HAS_DC_MOTOR || !HAS_ENCODER)
#error "HAS_DC_VELOCITY_PID requires HAS_DC_MOTOR=1 and HAS_ENCODER=1"
#endif

#ifndef HAS_MOTION_CONTROLLER
#define HAS_MOTION_CONTROLLER 0
#endif

// =============================================================================
// SENSOR FLAGS
// =============================================================================
#ifndef HAS_IMU
#define HAS_IMU 0
#endif

#ifndef HAS_LIDAR
#define HAS_LIDAR 0
#endif

#ifndef HAS_ULTRASONIC
#define HAS_ULTRASONIC 0
#endif

#ifndef HAS_LIMIT_SWITCH
#define HAS_LIMIT_SWITCH 0
#endif

#ifndef HAS_I2C
// Auto-enable I2C if IMU or LiDAR enabled
#define HAS_I2C (HAS_IMU || HAS_LIDAR)
#endif

// =============================================================================
// CONTROL FLAGS
// =============================================================================
#ifndef HAS_SIGNAL_BUS
#define HAS_SIGNAL_BUS 0
#endif

#ifndef HAS_CONTROL_KERNEL
#define HAS_CONTROL_KERNEL 0
#endif

// Control kernel requires signal bus
#if HAS_CONTROL_KERNEL && !HAS_SIGNAL_BUS
#error "HAS_CONTROL_KERNEL requires HAS_SIGNAL_BUS=1"
#endif

#ifndef HAS_OBSERVER
#define HAS_OBSERVER 0
#endif

#if HAS_OBSERVER && !HAS_SIGNAL_BUS
#error "HAS_OBSERVER requires HAS_SIGNAL_BUS=1"
#endif

#ifndef HAS_PID_CONTROLLER
// Auto-enable if control kernel enabled
#define HAS_PID_CONTROLLER HAS_CONTROL_KERNEL
#endif

#ifndef HAS_STATE_SPACE
#define HAS_STATE_SPACE 0
#endif

#ifndef HAS_CONTROL_MODULE
#define HAS_CONTROL_MODULE 0
#endif

#if HAS_CONTROL_MODULE && !HAS_CONTROL_KERNEL
#error "HAS_CONTROL_MODULE requires HAS_CONTROL_KERNEL=1"
#endif

// =============================================================================
// AUDIO FLAGS
// =============================================================================
#ifndef HAS_AUDIO
#define HAS_AUDIO 0
#endif

#ifndef HAS_DSP_CHAIN
#define HAS_DSP_CHAIN HAS_AUDIO
#endif

#ifndef HAS_MIC
#define HAS_MIC 0
#endif

#ifndef HAS_AUDIO_GRAPH
#define HAS_AUDIO_GRAPH 0
#endif

#ifndef HAS_BIQUAD
#define HAS_BIQUAD HAS_DSP_CHAIN
#endif

// =============================================================================
// SYSTEM FLAGS
// =============================================================================
#ifndef HAS_TELEMETRY
#define HAS_TELEMETRY 1
#endif

#ifndef HAS_HEARTBEAT
#define HAS_HEARTBEAT 1
#endif

#ifndef HAS_LOGGING
#define HAS_LOGGING 1
#endif

#ifndef HAS_IDENTITY
#define HAS_IDENTITY 0
#endif

#ifndef HAS_OTA
#define HAS_OTA 0
#endif

// OTA requires WiFi
#if HAS_OTA && !HAS_WIFI
#error "HAS_OTA requires HAS_WIFI=1"
#endif

#ifndef HAS_SAFETY_MANAGER
#define HAS_SAFETY_MANAGER 1  // Always recommended
#endif

#ifndef HAS_GPIO_MANAGER
#define HAS_GPIO_MANAGER 1
#endif

#ifndef HAS_PWM_MANAGER
#define HAS_PWM_MANAGER 1
#endif

#ifndef HAS_EVENT_BUS
#define HAS_EVENT_BUS 1  // Core dependency
#endif

#ifndef HAS_MESSAGE_ROUTER
#define HAS_MESSAGE_ROUTER 1
#endif

#ifndef HAS_COMMAND_HANDLER
#define HAS_COMMAND_HANDLER 1
#endif

// =============================================================================
// BINARY COMMANDS (for high-rate streaming)
// =============================================================================
#ifndef HAS_BINARY_COMMANDS
#define HAS_BINARY_COMMANDS 1
#endif

// =============================================================================
// DERIVED FLAGS (convenience macros)
// =============================================================================
#define HAS_ANY_MOTOR (HAS_DC_MOTOR || HAS_SERVO || HAS_STEPPER)
#define HAS_ANY_SENSOR (HAS_IMU || HAS_LIDAR || HAS_ULTRASONIC || HAS_ENCODER)
#define HAS_ANY_TRANSPORT (HAS_WIFI || HAS_BLE || HAS_UART_TRANSPORT || HAS_MQTT_TRANSPORT)
#define HAS_ANY_CONTROL (HAS_CONTROL_KERNEL || HAS_OBSERVER || HAS_SIGNAL_BUS)

// =============================================================================
// FEATURE SUMMARY (for debug output)
// =============================================================================
#if ENABLE_DEBUG_LOGS
#define FEATURE_FLAGS_SUMMARY() do { \
    DBG_PRINTLN("=== Feature Flags ==="); \
    DBG_PRINTF("Transport: WIFI=%d BLE=%d UART=%d MQTT=%d\n", HAS_WIFI, HAS_BLE, HAS_UART_TRANSPORT, HAS_MQTT_TRANSPORT); \
    DBG_PRINTF("Motor: DC=%d SERVO=%d STEPPER=%d ENCODER=%d MOTION=%d\n", HAS_DC_MOTOR, HAS_SERVO, HAS_STEPPER, HAS_ENCODER, HAS_MOTION_CONTROLLER); \
    DBG_PRINTF("Sensor: IMU=%d LIDAR=%d ULTRASONIC=%d\n", HAS_IMU, HAS_LIDAR, HAS_ULTRASONIC); \
    DBG_PRINTF("Control: SIGNAL_BUS=%d KERNEL=%d OBSERVER=%d\n", HAS_SIGNAL_BUS, HAS_CONTROL_KERNEL, HAS_OBSERVER); \
    DBG_PRINTF("System: TELEM=%d HEARTBEAT=%d LOG=%d OTA=%d\n", HAS_TELEMETRY, HAS_HEARTBEAT, HAS_LOGGING, HAS_OTA); \
} while(0)
#else
#define FEATURE_FLAGS_SUMMARY() ((void)0)
#endif
