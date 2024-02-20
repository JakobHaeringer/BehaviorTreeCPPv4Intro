/**
 * @file Command.cpp
 * @author Jakob Haeringer
 * @brief This file implements the output commands used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-13
 */
#include "planner/Command.hpp"

Command::Command(ros::NodeHandle& handle) {
    this->commandPub  = handle.advertise<bfmc_interface::NucleoCommand>("/output/actuator_command", 1);
    this->velocity    = VELOCITY_NEUTRAL;
    this->steering    = STEERING_NEUTRAL;
    this->stopVehicle = false;
}

Command::~Command() {}

void Command::publish() {
    if (this->stopVehicle) {
        commandMsg.command_type = COMMAND_BRAKE;
        commandMsg.value        = 0;
        commandPub.publish(commandMsg);
    }
    else {
        commandMsg.command_type = COMMAND_STEER;
        commandMsg.value        = this->steering;
        commandPub.publish(commandMsg);

        commandMsg.command_type = COMMAND_SPEED;
        commandMsg.value        = this->velocity;
        commandPub.publish(commandMsg);
    }
}
