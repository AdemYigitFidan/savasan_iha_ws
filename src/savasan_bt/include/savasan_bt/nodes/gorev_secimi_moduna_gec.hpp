#pragma once
#include <behaviortree_cpp/action_node.h>
#include <iostream>

class GorevSecimiModunaGec : public BT::SyncActionNode
{
public:
    GorevSecimiModunaGec(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[ACTION] GorevSecimiModunaGec -> SUCCESS (dummy)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
