#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <iostream>
#include <ros/ros.h>

using namespace BT;

// Node class - Action example
class FindBall : public BT::SyncActionNode {
public:
    explicit FindBall(const std::string& name) :
        BT::SyncActionNode(name, {}) {
    }

    BT::NodeStatus tick() override {
        ROS_INFO("FindBall");
        // int    i = 8;
        // double x = 5.0;
        // ROS_INFO_STREAM("ROS_INFO_STREAM: i" << i << "x:" << x);
        // ROS_WARN_STREAM("ROS_WARN_STREAM: i" << i << "x:" << x);
        // ROS_ERROR_STREAM("ROS_ERROR_STREAM: i" << i << "x:" << x);
        // ROS_DEBUG_STREAM("ROS_DEBUG_STREAM: i" << i << "x:" << x);
        // ROS_FATAL_STREAM("ROS_WARN_STREAM: i" << i << "x:" << x);
        return BT::NodeStatus::SUCCESS;
    }
};

// function - conditions
BT::NodeStatus BallFound() {
    ROS_INFO("BallFound");
    return BT::NodeStatus::SUCCESS;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bt_example");
    ros::NodeHandle nh;
    ros::Rate       loop_rate(2);

    BT::BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<FindBall>("FindBall");

    factory.registerSimpleCondition("BallFound", std::bind(BallFound));

    // Get XML path location from launch file
    std::string xml_file;
    nh.getParam("/bt_planner/xml_file", xml_file);
    // Create tree
    auto tree = factory.createTreeFromFile(xml_file); // Print structure of tree

    BT::printTreeRecursively(tree.rootNode());

    // Connect to Groot2 realtime monitor and specify Port (Standard Port is: 1667)
    BT::Groot2Publisher publisher(tree, 12345);

    // Execute the tree until it returns success or failure
    while (ros::ok()) {
        tree.tickWhileRunning();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}