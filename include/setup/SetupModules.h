#pragma once

#include "setup/ISetupModule.h"
#include <cstddef>

namespace mcu {
struct ServiceContext;
}

// =============================================================================
// SETUP MODULE MANIFEST
// =============================================================================
// This is the single source of truth for setup module ordering.
// To add a new setup module:
// 1. Create SetupXxx.cpp with your ISetupModule implementation
// 2. Add getSetupXxxModule() declaration below
// 3. Add it to the manifest in SetupManifest.cpp
// =============================================================================

// Individual setup module accessors
mcu::ISetupModule* getSetupWifiModule();
mcu::ISetupModule* getSetupOtaModule();
mcu::ISetupModule* getSetupSafetyModule();
mcu::ISetupModule* getSetupTransportModule();
mcu::ISetupModule* getSetupMotorsModule();
mcu::ISetupModule* getSetupSensorsModule();
mcu::ISetupModule* getSetupTelemetryModule();

/**
 * Get the complete setup module manifest.
 * Returns a null-terminated array of setup modules in execution order.
 * Critical modules are marked and will halt the system on failure.
 *
 * Order:
 * 1. WiFi - Network connectivity
 * 2. OTA - Over-the-air updates
 * 3. Safety - Mode manager, watchdogs (CRITICAL)
 * 4. Motors - Motor drivers, motion controller
 * 5. Sensors - IMU, encoders, ultrasonic
 * 6. Transport - Router, commands, host (CRITICAL)
 * 7. Telemetry - Telemetry providers
 */
mcu::ISetupModule** getSetupManifest();

/**
 * Get the number of modules in the manifest.
 */
size_t getSetupManifestSize();
