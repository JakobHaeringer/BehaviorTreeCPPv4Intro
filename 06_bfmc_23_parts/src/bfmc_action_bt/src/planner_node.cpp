#include "BTClasses.hpp"
#include "Reset.hpp"
#include "RoutePlanning.hpp"
#include "StartUp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ros/package.h>
#include <ros/ros.h>

#define ROS_RATE 2

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_bfmc_23");
    ros::NodeHandle nh;
    ros::Rate       loop_rate(ROS_RATE);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ApproachStopLine>("ApproachStopLine");
    factory.registerNodeType<ChangeLane>("ChangeLane");
    factory.registerNodeType<CrossIntersection>("CrossIntersection");
    factory.registerNodeType<Crosswalk>("Crosswalk");
    factory.registerNodeType<FollowCar>("FollowCar");
    factory.registerNodeType<FollowLane>("FollowLane");
    factory.registerNodeType<ParkCar>("ParkCar");
    factory.registerNodeType<PedestrianOnRoad>("PedestrianOnRoad");
    factory.registerNodeType<Reset>("Reset");
    factory.registerNodeType<Roundabout>("Roundabout");
    factory.registerNodeType<RoutePlanning>("RoutePlanning");
    factory.registerNodeType<StartUp>("StartUp");
    factory.registerNodeType<Stop>("Stop");
    factory.registerNodeType<TrafficLight>("TrafficLight");

    // Get XML path location from launch file
    std::string xml_file;
    nh.getParam("/planner/xml_file", xml_file);
    // Create tree
    auto tree = factory.createTreeFromFile(xml_file); // Print structure of tree

    BT::printTreeRecursively(tree.rootNode());

    // Connect to Groot2 realtime monitor and specify Port (Standard Port is: 1667)
    BT::Groot2Publisher publisher(tree, 12345);

    while (ros::ok()) {
        tree.tickWhileRunning();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
