#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <chrono>
#include <iostream>

using namespace BT;
using namespace std::chrono_literals;

bool run_inf = false;

// Node class - Action example
class FindBall : public BT::SyncActionNode {
public:
    explicit FindBall(const std::string& name) :
        BT::SyncActionNode(name, {}) {
    }

    BT::NodeStatus tick() override {
        std::cout << "FindBall" << std::endl;
        std::this_thread::sleep_for(5s);
        return BT::NodeStatus::SUCCESS;
    }
};

// Node class - Action example
class GrabBall : public BT::SyncActionNode {
public:
    explicit GrabBall(const std::string& name) :
        BT::SyncActionNode(name, {}) {
    }

    BT::NodeStatus tick() override {
        std::cout << "GrabBall" << std::endl;
        std::this_thread::sleep_for(5s);
        return BT::NodeStatus::SUCCESS;
    }
};

// Node class - Action example
class ThrowBall : public BT::SyncActionNode {
public:
    explicit ThrowBall(const std::string& name) :
        BT::SyncActionNode(name, {}) {
    }

    BT::NodeStatus tick() override {
        std::cout << "ThrowBall" << std::endl;
        std::this_thread::sleep_for(5s);
        return BT::NodeStatus::FAILURE;
    }
};

// Node class - Action example
class Reset : public BT::SyncActionNode {
public:
    explicit Reset(const std::string& name) :
        BT::SyncActionNode(name, {}) {
    }

    BT::NodeStatus tick() override {
        std::cout << "Reset" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// function - conditions
BT::NodeStatus BallFound() {
    std::cout << "BallFound" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// function - conditions
BT::NodeStatus BallGrabbed() {
    std::cout << "BallGrabbed" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// function - conditions
BT::NodeStatus BallThrown() {
    std::cout << "BallThrown" << std::endl;
    return BT::NodeStatus::FAILURE;
}

int main() {
    BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<FindBall>("FindBall");
    factory.registerNodeType<GrabBall>("GrabBall");
    factory.registerNodeType<ThrowBall>("ThrowBall");
    factory.registerNodeType<Reset>("Reset");

    factory.registerSimpleCondition("BallFound", std::bind(BallFound));
    factory.registerSimpleCondition("BallGrabbed", std::bind(BallGrabbed));
    factory.registerSimpleCondition("BallThrown", std::bind(BallThrown));

    // Create tree
    auto tree = factory.createTreeFromFile("../../01_GrootProject/SimpleBallExample.xml");
    // Print structure of tree
    BT::printTreeRecursively(tree.rootNode());

    // Connect to Groot2 -- needs Groot2 Pro
    // BT::Groot2Publisher publisher(tree);

    if (run_inf) {
        while (1) {
            tree.tickWhileRunning();
        }
    }
    else tree.tickWhileRunning();

    return 0;
}