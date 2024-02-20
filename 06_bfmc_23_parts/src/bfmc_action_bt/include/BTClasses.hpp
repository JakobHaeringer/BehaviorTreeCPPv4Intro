#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ros/ros.h>

using namespace BT;

class ApproachStopLine : public BT::StatefulActionNode {
public:
    explicit ApproachStopLine(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    // On Start could be used to check if stop line etc. is available - yes=return RUNNING no=return SUCCESS
    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("ApproachStopLine On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("ApproachStopLine On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};

class ChangeLane : public BT::StatefulActionNode {
public:
    explicit ChangeLane(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("ChangeLane On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("ChangeLane On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};

class CrossIntersection : public BT::StatefulActionNode {
public:
    explicit CrossIntersection(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("CrossIntersection On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("CrossIntersection On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};

class Crosswalk : public BT::StatefulActionNode {
public:
    explicit Crosswalk(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("Crosswalk On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("Crosswalk On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};

class FollowCar : public BT::StatefulActionNode {
public:
    explicit FollowCar(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("FollowCar On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("FollowCar On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};

class FollowLane : public BT::StatefulActionNode {
public:
    explicit FollowLane(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("FollowLane On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("FollowLane On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};
        
class ParkCar : public BT::StatefulActionNode {
public:
    explicit ParkCar(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("ParkCar On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("ParkCar On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};

class PedestrianOnRoad : public BT::StatefulActionNode {
public:
    explicit PedestrianOnRoad(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("PedestrianOnRoad On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("PedestrianOnRoad On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};
                
class Roundabout : public BT::StatefulActionNode {
public:
    explicit Roundabout(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("Roundabout On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("Roundabout On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};
                    
class Stop : public BT::StatefulActionNode {
public:
    explicit Stop(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("Stop On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("Stop On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};
                        
class TrafficLight : public BT::StatefulActionNode {
public:
    explicit TrafficLight(const std::string& name, const NodeConfig& config) :
        BT::StatefulActionNode(name, config) {
    }

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<bool>("error")};
    }

    BT::NodeStatus onStart() override {
        ROS_INFO_STREAM("TrafficLight On Start");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        ROS_INFO_STREAM("TrafficLight On Running");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
    }
};
