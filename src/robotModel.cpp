#include "robotModel.hpp"
#include "robotModel_utils.hpp"

#include "yaml-cpp/yaml.h"
#include <iostream>

#include <string>
#include <vector>

// #include "tf_Graph.hpp"

namespace RobotModel
{

using namespace std;
using namespace boost;

void operator<<(vector<Joint_Link_pair> &jlSet, const YAML::Node &node)
{
    Joint_Link_pair p;
    for (auto it = node.begin(); it != node.end(); ++it)
    {
        p << *it;
        jlSet.push_back(p);
    }
}

RobotModel::RobotModel(YAML::Node node)
{
    // ControlableJoints
    ControlableJoints = parseYAMLList<string>(node["ControlableJoints"]);
    JointMap = parseYAMLMap<Joint>(node["joint_map"]);
    LinkMap = parseYAMLMap<Link>(node["link_map"]);
    ParentMap = parseYAMLMap<Joint_Link_pair>(node["parent_map"]);
    ChildMap = parseYAMLMap<vector<Joint_Link_pair>>(node["child_map"]);

    build_frame_Tree();
}

void RobotModel::build_frame_Tree()
{
    tf_Graph.build(JointMap, LinkMap);
}

} // namespace RobotModel
