/**
 * @file Actions.hpp
 * @author Jakob Haeringer and Noah Koehler
 * @brief The header file for actions used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-13
 */

#ifndef _ACTIONS_HPP_
#define _ACTIONS_HPP_

#include "planner/AStar.hpp"
#include "planner/Command.hpp"
#include "planner/Environment.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#define _USE_MATH_DEFINES     ///< Some mathematical defines like PI etc.
#define TRANSITION_FACTOR 100 ///< [m/cm] Factor to scale the received distance from m to cm (better fitting for steering angle)

using namespace BT;

/** @brief This struct defines various time parameters specified in seconds [s] for the actions.*/
struct TimeToWait {
    double startUp;    ///< Time for the start up.
    double brake;      ///< Time for braking.
    double stop;       ///< Time to wait at a stop sign.
    double pedestrian; ///< Maximum time to wait in pedestrian state.
};

/** @brief This class contains all functions for the vehicle states, auxiliary functions and an instance of the AStar class to get global map data.
 *
 * The vehicle states include the following: StartUp, FollowLane, Crosswalk, Pedestrian, Stop(Sign), PriorityRoad(Sign), Intersection, RoutePlanning, SingleOneWay(Street), ParkCar, ChangeLane and TrafficLight.
 * There are also two auxiliary functions integrated, namely getDirectionToDrive and resetSystem.
 * Within the class there is an instance of the AStar class, which is used to get global map data and to plan the shortest route to get to different scenarios.
 */
class Actions {
public:
    /** @brief Constructs a new Actions object and loads the map data into the AStar instance.*/
    Actions();
    /** @brief Destroy the Actions object.*/
    ~Actions();
    /**
     * @brief The function waits for all processes to start, plans the route, and drives when the traffic light is green.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void StartUp(Environment* env, Command* cmd);
    /**
     * @brief Sets the steering angle and velocity based on the lane detection and performs an emergency braking if a vehicle is within a safety distance.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void FollowLane(Environment* env, Command* cmd);
    /**
     * @brief Regulates the behavior of the vehicle approaching a crosswalk and changes the state depending on the detected/ not detected pedestrian.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void Crosswalk(Environment* env, Command* cmd);
    /**
     * @brief Regulates the behavior of the vehicle when encountering pedestrians in the crosswalk and on the roadway.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void Pedestrian(Environment* env, Command* cmd);
    /**
     * @brief Regulates the behavior of the vehicle approaching an intersection with a stop sign and brings the car to a stop.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void Stop(Environment* env, Command* cmd);
    /**
     * @brief Regulates the behavior of the vehicle approaching an intersection with a priority sign.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void PriorityRoad(Environment* env, Command* cmd);
    /**
     * @brief Sets a sequence of static steering commands to cross the intersection.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void Intersection(Environment* env, Command* cmd);
    /**
     * @brief Calculate the shortest route between two map nodes using the AStar algorithm and add the summed distances for each node.
     * @param env Pointer to the environment object.
     */
    void RoutePlanning(Environment* env);
    /**
     * @brief Turns the vehicle from the highway to the one-way street and then follows a car and controls the velocity based on the distance to the car ahead.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void SingleOneWay(Environment* env, Command* cmd);
    /**
     * @brief Finds the free parking spot (crosswise parking) and sets a sequence of steering and velocity commands to perform a parking maneuver.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void ParkCar(Environment* env, Command* cmd);
    /**
     * @brief Sets a sequence of steering and velocity commands to switch between lanes.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void ChangeLane(Environment* env, Command* cmd);
    /**
     * @brief Stops the vehicle at a red light and/or allows it to proceed at a green light.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void TrafficLight(Environment* env, Command* cmd);
    /**
     * @brief Sets a sequence of steering and velocity commands to drive through the roundabout.
     * @param env Pointer to the environment object.
     * @param cmd Pointer to the command object.
     */
    void Roundabout(Environment* env, Command* cmd);
    /**
     * @brief Determines the direction in which the vehicle must travel within the current intersection (left, right, straight ahead) based on the planned route.
     * @param env Pointer to the environment object.
     */
    void getDirectionToDrive(Environment* env);
    /**
     * @brief Resets certain environment variables.
     * @param env Pointer to the environment object.
     */
    void resetSystem(Environment* env);

    TimeToWait duration; ///< Struct that holds various time parameters specified in seconds [s] for the actions.

private:
    AStarAlgorithm AStar; ///< Class instance used to run an AStar algorithm to specify a route plan and to access data in the global map.
    float          P;     ///< Value used to store the current P-value of the lateral P-controller.
    float          oldP;  ///< Value used to store the previous P-value of the lateral P-controller.
};
#endif
