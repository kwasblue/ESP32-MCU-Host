#pragma once
#include <cstdint>
#include "core/Event.h"

// Forward declaration
namespace mcu {
struct ServiceContext;
}

/**
 * Base interface for all modules.
 *
 * Modules follow a lifecycle:
 * 1. Construction (default or with dependencies)
 * 2. init(ServiceContext&) - receive dependencies (for self-registered modules)
 * 3. setup() - one-time initialization
 * 4. loop(now_ms) - called repeatedly from main loop
 * 5. handleEvent(evt) - optional event handling
 *
 * For self-registration using REGISTER_MODULE macro:
 * - Use default constructor
 * - Override init() to receive dependencies from ServiceContext
 *
 * For manual registration via MCUHost::addModule():
 * - Take dependencies in constructor
 * - init() is still called but may be empty
 */
class IModule {
public:
    virtual ~IModule() = default;

    /**
     * Initialize module with service dependencies.
     * Called after all modules are registered, before setup().
     * Self-registered modules should override this to get dependencies.
     */
    virtual void init(mcu::ServiceContext& ctx) { (void)ctx; }

    /**
     * One-time setup after init(). Called before loop() starts.
     */
    virtual void setup() {}

    /**
     * Called repeatedly from main loop.
     * @param now_ms Current time in milliseconds
     */
    virtual void loop(uint32_t now_ms) { (void)now_ms; }

    /**
     * Get module name for debugging.
     */
    virtual const char* name() const = 0;

    /**
     * Handle events from EventBus (optional).
     */
    virtual void handleEvent(const Event& evt) { (void)evt; }

    /**
     * Get module priority (lower = earlier in init/setup/loop order).
     * Default is 100. Critical modules should use < 50.
     */
    virtual int priority() const { return 100; }
};
