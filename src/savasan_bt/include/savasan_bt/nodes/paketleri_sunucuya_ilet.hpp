#pragma once
#include <behaviortree_cpp/action_node.h>
#include <iostream>

class PaketleriSunucuyaIlet : public BT::SyncActionNode
{
public:
    PaketleriSunucuyaIlet(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[ACTION] PaketleriSunucuyaIlet -> SUCCESS (dummy)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
