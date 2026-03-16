#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <iostream>

class SinirVeyaIrtifaAsildiMi : public BT::ConditionNode
{
public:
    SinirVeyaIrtifaAsildiMi(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[CONDITION] SinirVeyaIrtifaAsildiMi -> FAILURE (dummy)" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};
