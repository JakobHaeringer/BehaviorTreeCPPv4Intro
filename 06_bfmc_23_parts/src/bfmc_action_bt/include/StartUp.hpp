#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ros/ros.h>

using namespace BT;

class StartUp : public BT::StatefulActionNode {
public:
    explicit StartUp(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("StartUp On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("StartUp On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};