#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ros/ros.h>

using namespace BT;

class Reset : public BT::StatefulActionNode {
public:
    explicit Reset(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("error")};
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        BT::Expected<bool> msg = getInput<bool>("error");
        // bool               value = msg.value();
        // ROS_INFO_STREAM("Received error msg" << value);
        if (msg) {
            ROS_INFO_STREAM("Reset");
            BT::TreeNode::setOutput("error", false);
        }
        else ROS_INFO_STREAM("No Reset");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
    }
};