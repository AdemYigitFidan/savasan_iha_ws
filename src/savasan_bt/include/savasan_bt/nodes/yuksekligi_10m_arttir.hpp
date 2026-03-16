#pragma once
#include <behaviortree_cpp/action_node.h>
#include <iostream>

class Yuksekligi10mArttir : public BT::SyncActionNode
{
public:
    Yuksekligi10mArttir(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[ACTION] Yuksekligi10mArttir -> SUCCESS (dummy)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
