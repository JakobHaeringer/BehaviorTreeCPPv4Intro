/**
 * @file planner_node.cpp
 * @author Jakob Haeringer
 * @brief This file manages the Environment, Action and Command classes and is responsible for the behavior planning of the vehicle used in the BFMC 2023.
 * @version 1.0
 * @date 2023-07-18
 */
#include "planner/Actions.hpp"
#include "planner/Command.hpp"
#include "planner/Environment.hpp"
#include <ros/package.h>
#include <ros/ros.h>

/** @brief Specifies the frequency of the behavior planning - 33ms per iteration.*/
#define ROS_RATE 30 // TODO: Test ROS Rate - Test with 30 works - Maybe test faster again - Benefit when faster than lanedetect?

/**
 * @brief Main function for handling the behavior planning of the vehicle.
 *
 * From each class one object is instantiated as well as a node handler for ROS communication.
 * Special variables are set with the launch files parameter. The plannerNode runs with 30 Hz.
 * During each loop it is checked whether a hard reset is to be performed, then the current node ID is determined from the planned route.
 * Based on the vehicle's input, a state is detected that leads to the execution of the corresponding action.
 * After executing the action the plannerNode publishes commands (steering, velocity) and the EgoPose.
 * @param argc
 * @param argv
 * @return int 0
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::Rate       loop_rate(ROS_RATE);
    Command         cmd(nh);
    Environment     env(nh);
    Actions         act;

    int startState;
    nh.getParam("/planner/startState", startState);
    env.currentState = static_cast<State>(startState);

    // Testing purposes
    nh.getParam("/planner/distanceToDrive", env.distanceToDrive);
    nh.getParam("/planner/currentNodeID", env.currentNodeID);
    nh.getParam("/planner/targetNodeID", env.targetNodeID);

    // TODO: Initialize complete currentPose with StartValues also node ID - rename currentNodeID to startNodeID
    nh.getParam("/planner/initialPoseX", env.currentPose.x);
    nh.getParam("/planner/initialPoseY", env.currentPose.y);
    nh.getParam("/planner/initialPoseYaw", env.currentPose.yaw);
    nh.getParam("/planner/initialPoseYaw", env.initialCarYaw);

    // nh.getParam("/planner/raceMode", env.raceMode);

    nh.getParam("/planner/velocityDefault", cmd.velocityDefault);
    nh.getParam("/planner/velocityReduced", cmd.velocityReduced);
    nh.getParam("/planner/velocityParking", cmd.velocityParking);
    nh.getParam("/planner/velocityHighWay", cmd.velocityHighWay);
    nh.getParam("/planner/timings/startUp", act.duration.startUp);
    nh.getParam("/planner/timings/brake", act.duration.brake);
    nh.getParam("/planner/timings/stop", act.duration.stop);
    nh.getParam("/planner/timings/pedestrian", act.duration.pedestrian);

    // Time Measurement
    ros::Time tStart;
    ros::Time tEnd;

    while (ros::ok()) {
        // Measure time
        tStart = ros::Time::now();

        // if (env.hardReset) act.resetSystem(&env);
        while (env.travelledDistance >= env.plannedRoute[env.globalRouteIterator].totalDistance) {
            if (env.globalRouteIterator >= env.plannedRoute.size() - 1) {
                break;
            }
            else {
                env.globalRouteIterator++;
            }
        }
        env.currentPose.nodeID = env.plannedRoute[env.globalRouteIterator].nodeID;
        env.egoPoseMsg.nodeID  = env.currentNodeID;
        ROS_INFO("State: %d, X: %f, Y: %f, Yaw: %f, Steer: %f, Velo: %f, coeff: %f", static_cast<int>(env.currentState), env.currentPose.x, env.currentPose.y, env.currentPose.yaw * 180 / M_PI, cmd.steering, cmd.velocity, env.curveCoeff);

        switch (env.currentState) {
        case State::START_UP:
            act.StartUp(&env, &cmd);
            break;
        case State::FOLLOW_LANE:
            act.FollowLane(&env, &cmd);
            break;
        case State::INTERSECTION:
            act.Intersection(&env, &cmd);
            break;
        case State::STOP:
            act.Stop(&env, &cmd);
            break;
        case State::CROSSWALK:
            act.Crosswalk(&env, &cmd);
            break;
        case State::PARKING:
            act.ParkCar(&env, &cmd);
            break;
        case State::ROUTE_PLANNING:
            act.RoutePlanning(&env);
            break;
        case State::PRIORITY_ROAD:
            act.PriorityRoad(&env, &cmd);
            break;
        case State::PEDESTRIAN:
            act.Pedestrian(&env, &cmd);
            break;
        case State::SINGLE_ONE_WAY:
            act.SingleOneWay(&env, &cmd);
            break;
        case State::CHANGE_LANE:
            act.ChangeLane(&env, &cmd);
            break;
        case State::TRAFFIC_LIGHT:
            act.TrafficLight(&env, &cmd);
            break;
        case State::ROUNDABOUT:
            act.Roundabout(&env, &cmd);
            break;
        case State::FINISH:
            cmd.stopVehicle = true;
            break;
        default:
            cmd.stopVehicle = true;
            break;
        }

        cmd.publish();
        env.publish();
        ros::spinOnce();

        // Measure time
        tEnd = ros::Time::now();
        // double elapsedTime = (tEnd - tStart).toSec() * 1000; // conversion to ms
        // ROS_INFO("PLANNER\tTime passed: %f", elapsedTime);

        loop_rate.sleep();
    }
    return 0;
}
