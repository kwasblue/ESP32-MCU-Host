#include "setup/SetupControlTask.h"
#include "core/ServiceContext.h"
#include "module/ControlModule.h"
#include "command/ModeManager.h"
#include "motor/MotionController.h"
#include "loop/LoopFunctions.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace mcu {

// Task state
static TaskHandle_t g_controlTaskHandle = nullptr;
static ServiceContext* g_taskCtx = nullptr;
static ControlTaskConfig g_taskConfig;
static volatile bool g_taskRunning = false;

// Statistics (volatile for cross-task access)
static volatile ControlTaskStats g_stats;

// The FreeRTOS control task
static void controlTaskFunc(void* param) {
    ServiceContext* ctx = static_cast<ServiceContext*>(param);

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / g_taskConfig.rate_hz);

    g_taskRunning = true;

    Serial.printf("[CTRL_TASK] Started on Core %d at %d Hz (period=%d ticks)\n",
                  xPortGetCoreID(), g_taskConfig.rate_hz, (int)period);

    for (;;) {
        uint32_t start_us = micros();
        uint32_t now_ms = millis();

        // Compute dt based on configured rate
        float dt_s = 1.0f / g_taskConfig.rate_hz;

        // Run full control loop (encoder PID, motion controller)
        runControlLoop(*ctx, now_ms, dt_s);

        // Run control module (PID/LQR slots, observers)
        if (ctx->control) {
            ctx->control->loop(now_ms);
        }

        // Compute execution time
        uint32_t exec_us = micros() - start_us;
        g_stats.last_exec_us = exec_us;
        g_stats.iterations++;

        if (exec_us > g_stats.max_exec_us) {
            g_stats.max_exec_us = exec_us;
        }

        // Check for overrun (execution took longer than period)
        uint32_t period_us = 1000000 / g_taskConfig.rate_hz;
        if (exec_us > period_us) {
            g_stats.overruns++;
        }

        // Wait until next period - vTaskDelayUntil provides precise timing
        vTaskDelayUntil(&lastWake, period);
    }
}

bool startControlTask(ServiceContext& ctx, const ControlTaskConfig& config) {
    if (g_controlTaskHandle != nullptr) {
        Serial.println("[CTRL_TASK] Already running");
        return false;
    }

    // Validate config
    if (config.rate_hz < 10 || config.rate_hz > 1000) {
        Serial.printf("[CTRL_TASK] Invalid rate: %d Hz (must be 10-1000)\n", config.rate_hz);
        return false;
    }

    g_taskCtx = &ctx;
    g_taskConfig = config;

    // Reset stats (individual fields for volatile)
    g_stats.iterations = 0;
    g_stats.max_exec_us = 0;
    g_stats.overruns = 0;
    g_stats.last_exec_us = 0;

    // Create the task pinned to specified core
    BaseType_t result = xTaskCreatePinnedToCore(
        controlTaskFunc,
        "ControlTask",
        config.stack_size,
        &ctx,
        config.priority,
        &g_controlTaskHandle,
        config.core
    );

    if (result != pdPASS) {
        Serial.println("[CTRL_TASK] Failed to create task");
        g_controlTaskHandle = nullptr;
        return false;
    }

    return true;
}

void stopControlTask() {
    if (g_controlTaskHandle == nullptr) {
        return;
    }

    Serial.println("[CTRL_TASK] Stopping...");

    // Delete the task
    vTaskDelete(g_controlTaskHandle);
    g_controlTaskHandle = nullptr;
    g_taskRunning = false;
    g_taskCtx = nullptr;

    Serial.println("[CTRL_TASK] Stopped");
}

bool isControlTaskRunning() {
    return g_taskRunning && g_controlTaskHandle != nullptr;
}

ControlTaskStats getControlTaskStats() {
    // Return a copy (volatile reads)
    ControlTaskStats stats;
    stats.iterations = g_stats.iterations;
    stats.max_exec_us = g_stats.max_exec_us;
    stats.overruns = g_stats.overruns;
    stats.last_exec_us = g_stats.last_exec_us;
    return stats;
}

void resetControlTaskStats() {
    g_stats.iterations = 0;
    g_stats.max_exec_us = 0;
    g_stats.overruns = 0;
    g_stats.last_exec_us = 0;
}

} // namespace mcu
