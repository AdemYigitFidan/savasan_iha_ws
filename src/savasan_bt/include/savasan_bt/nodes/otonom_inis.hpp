#pragma once
#include <behaviortree_cpp/action_node.h>
#include <iostream>

class OtonomInis : public BT::SyncActionNode
{
public:
    OtonomInis(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[ACTION] OtonomInis -> SUCCESS (dummy)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
