#pragma once
#include <behaviortree_cpp/action_node.h>
#include <iostream>

class Yuz100mOlanaKadarTirman : public BT::SyncActionNode
{
public:
    Yuz100mOlanaKadarTirman(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[ACTION] Yuz100mOlanaKadarTirman -> SUCCESS (dummy)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
