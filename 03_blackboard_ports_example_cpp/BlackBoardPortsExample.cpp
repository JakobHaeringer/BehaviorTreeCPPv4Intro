#include "behaviortree_cpp/bt_factory.h"
#include <cmath>
#include <iostream>

using namespace BT;
using namespace std::chrono_literals;

bool run_inf = true;

// Node class - Action example
class SetSteering : public BT::SyncActionNode {
public:
    explicit SetSteering(const std::string& name, const NodeConfig& config) :
        BT::SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<float>("steering")};
    }

    BT::NodeStatus tick() override {
        std::cout << "--- SetSteering ---" << std::endl;
        float steering_value;
        std::cout << "Please enter a steering value: ";
        std::cin >> steering_value;
        BT::TreeNode::setOutput("steering", steering_value);
        return BT::NodeStatus::SUCCESS;
    }
};

// Node class - Action example
class SendSteering : public BT::SyncActionNode {
public:
    explicit SendSteering(const std::string& name, const NodeConfig& config) :
        BT::SyncActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<float>("steering"), BT::InputPort<bool>("steering_changed")};
    }

    BT::NodeStatus tick() override {
        std::cout << "--- SendSteering ---" << std::endl;

        if (getInput<bool>("steering_changed").value()) {
            BT::Expected<float> msg = getInput<float>("steering");
            // Check if expected is valid. If not, throw its error
            if (!msg) {
                throw BT::RuntimeError("missing required input [message]: ",
                                       msg.error());
            }
            // use the method value() to extract the valid message.
            std::cout << "Send the set steering value: " << msg.value() << std::endl;
        }
        else std::cout << "Steering angle hasn't changed!" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

// Node class - Condition example
class SteeringIsZero : public BT::ConditionNode {
public:
    explicit SteeringIsZero(const std::string& name, const NodeConfig& config) :
        BT::ConditionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<float>("steering"), BT::OutputPort<bool>("steering_changed")};
    }

    BT::NodeStatus tick() override {
        std::cout << "--- SteeringIsZero ---" << std::endl;

        BT::Expected<float> msg = getInput<float>("steering");
        if (m_steering_angle != INFINITY && m_steering_angle == msg.value()) {
            BT::TreeNode::setOutput("steering_changed", false);
        }
        else BT::TreeNode::setOutput("steering_changed", true);

        m_steering_angle = msg.value();
        if (msg.value() == 0) return BT::NodeStatus::SUCCESS;
        else return BT::NodeStatus::FAILURE;
    }

private:
    float m_steering_angle = INFINITY;
};

int main() {
    BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<SendSteering>("SendSteering");
    factory.registerNodeType<SetSteering>("SetSteering");
    factory.registerNodeType<SteeringIsZero>("SteeringIsZero");

    // Create tree
    auto tree = factory.createTreeFromFile("../../01_GrootProject/Blackboard_Ports.xml");
    // Print structure of tree
    std::cout << "Structure of the tree" << std::endl;
    BT::printTreeRecursively(tree.rootNode());

    // Connect to Groot2 -- needs Groot2 Pro
    // BT::Groot2Publisher publisher(tree);

    std::cout << "Start to tick the tree" << std::endl;
    if (run_inf) {
        while (1) {
            tree.tickWhileRunning();
        }
    }
    else tree.tickWhileRunning();

    return 0;
}