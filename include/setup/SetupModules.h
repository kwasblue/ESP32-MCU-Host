#pragma once

#include "setup/ISetupModule.h"

namespace mcu {
struct ServiceContext;
}

// Setup module accessors
mcu::ISetupModule* getSetupWifiModule();
mcu::ISetupModule* getSetupOtaModule();
mcu::ISetupModule* getSetupSafetyModule();
mcu::ISetupModule* getSetupTransportModule();
mcu::ISetupModule* getSetupMotorsModule();
mcu::ISetupModule* getSetupSensorsModule();
mcu::ISetupModule* getSetupTelemetryModule();
