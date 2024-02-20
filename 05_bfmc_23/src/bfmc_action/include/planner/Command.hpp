/**
 * @file Command.hpp
 * @author Jakob Haeringer
 * @brief The header file for the output commands used in the BFMC 2023.
 *
 * @details Config file for steering and dc motor constraints on the Nucleo Board in BFMC/Embedded_Platform/src/main.cpp. \n
 * Steering constraints -22° bis 22° because of the chassis - -23 to +23 on Nucleo Board. \n
 * Velocity constraints - PWM signal for DC Motor -0.3 to 0.3 on Nucleo Board.
 * @version 1.0
 * @date 2023-07-13
 */
#ifndef _COMMAND_HPP_
#define _COMMAND_HPP_

#include "bfmc_interface/NucleoCommand.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#define STEERING_NEUTRAL   0.0f   ///< [°] Neutral steering angle.
#define STEERING_MAX_LEFT  -22.0f ///< [°] Maximum left steering angle.
#define STEERING_MAX_RIGHT 22.0f  ///< [°] Maximum right steering angle.

#define VELOCITY_BRAKING       -0.2f  ///< [PWM] Velocity for maximum braking or reversing.
#define VELOCITY_LIGHT_BRAKING -0.05f ///< [PWM] Velocity for light braking.
#define VELOCITY_NEUTRAL       0.0f   ///< [PWM] Neutral velocity.
#define VELOCITY_MIN_DRIVEABLE 0.09f  ///< [PWM] Minimal driveable velocity.
#define VELOCITY_MAX           0.2f   ///< [PWM] Maximum velocity.

/** @brief Enumeration containing the command types.*/
enum COMMANDS {
    COMMAND_SPEED = 1, ///< Specifies that the message sent to the Nucleo contains a velocity value.
    COMMAND_STEER = 2, ///< Specifies that the message sent to the Nucleo contains a steering signal.
    COMMAND_BRAKE = 3  ///< Specifies that the message sent to the Nucleo contains a signal to stop the vehicle.
};

    /** @brief This class contains the publisher for the steering and velocity commands and holds various velocity values.*/
    class Command {
    public:
        /**
         * @brief Construct a new Command object and initialize publisher as well as variables which are not set in the launch file.
         * @param handle Nodehandler for creating the ROS publishers and subscribers.
         */
        Command(ros::NodeHandle& handle);
        /** @brief Destroy the Command object. */
        ~Command();
        /** @brief Publishes either a command to stop the vehicle or a steering and velocity command.*/
        void publish();

        float velocityDefault; ///< Holds the default velocity value specified in the launch file.
        float velocityParking; ///< Holds the parking velocity value specified in the launch file.
        float velocityReduced; ///< Holds the reduced velocity value specified in the launch file.
        float velocityHighWay; ///< Holds the highway velocity value specified in the launch file.

        float velocity;    ///< Holds the value of the currently desired velocity.
        float steering;    ///< Holds the value of the currently desired steering angle.
        bool  stopVehicle; ///< Holds the boolean value whether the vehicle should be stopped.

    private:
        ros::Publisher                commandPub; ///< The publisher object that is used to publish the "utils::nucleocommand" message to the topic "/automobile/command".
        bfmc_interface::NucleoCommand commandMsg; ///< The message object that is used as a template for the ROS nucleocommand message.
    };
