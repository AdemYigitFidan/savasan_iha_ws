#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <iostream>

class KirmiziAlanaMiGirdin : public BT::ConditionNode
{
public:
    KirmiziAlanaMiGirdin(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[CONDITION] KirmiziAlanaMiGirdin -> FAILURE (dummy)" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};
