// include/command/CommandRegistry.h
// Central dispatcher that routes commands to registered handlers

#pragma once

#include <vector>
#include <string>
#include <ArduinoJson.h>
#include "command/ICommandHandler.h"
#include "command/CommandContext.h"
#include "command/BinaryCommands.h"
#include "core/EventBus.h"
#include "core/Event.h"
#include "core/Messages.h"
#include "core/Debug.h"

// Forward declarations
class ModeManager;
class MotionController;
class ControlModule;

/**
 * Command Registry - Central dispatcher for all commands.
 *
 * Replaces the monolithic CommandHandler with a plugin-based architecture.
 * Each domain (motors, sensors, control, etc.) registers its own handler.
 */
class CommandRegistry {
public:
    CommandRegistry(EventBus& bus, ModeManager& mode, MotionController& motion);

    /**
     * Register a command handler.
     * Handlers are checked in registration order.
     */
    void registerHandler(ICommandHandler* handler);

    /**
     * Setup event subscriptions.
     */
    void setup();

    /**
     * Set optional control module (for binary commands).
     */
    void setControlModule(ControlModule* cm) { controlModule_ = cm; }

    /**
     * Set motion controller reference (for binary commands).
     */
    void setMotionController(MotionController* mc) { motion_ = mc; }

    /**
     * Process incoming JSON command.
     */
    void onJsonCommand(const std::string& jsonStr);

    /**
     * Process incoming binary command.
     */
    void onBinaryCommand(const std::vector<uint8_t>& binData);

    /**
     * Get the command context (for handlers that need it).
     */
    CommandContext& context() { return ctx_; }

private:
    // Event handling
    static void handleEventStatic(const Event& evt);
    void handleEvent(const Event& evt);
    static CommandRegistry* s_instance;

    // Find handler for command type
    ICommandHandler* findHandler(CmdType cmd);

    // Members
    std::vector<ICommandHandler*> handlers_;
    CommandContext ctx_;
    MotionController* motion_ = nullptr;
    ControlModule* controlModule_ = nullptr;
};
