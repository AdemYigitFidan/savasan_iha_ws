#pragma once
#include <behaviortree_cpp/action_node.h>
#include <iostream>

class OtonomKalkis : public BT::SyncActionNode
{
public:
    OtonomKalkis(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[ACTION] OtonomKalkis -> SUCCESS (dummy)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
