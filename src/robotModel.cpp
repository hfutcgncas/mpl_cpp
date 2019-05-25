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
using namespace tf_Graph;

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
    
    for (auto item : LinkMap)
    {
        vertex_descriptor_t v = boost::add_vertex( *dynamic_cast<Frame*>(&item.second), tf_tree.g);
        tf_tree.Vmap.insert(pair<string, vertex_descriptor_t>(item.first, v));
       
        // std::cout<<tf_tree.g[v].name <<endl;
    }

    for (auto item : JointMap)
    {
        auto v1 = tf_tree.Vmap[item.second.parent];
        auto v2 = tf_tree.Vmap[item.second.child];

        // Add edges

        std::pair<edge_descriptor_t, bool> e = boost::add_edge(v1, v2, tf_tree.g);
        tf_tree.g[e.first] =  *dynamic_cast<TF*>(&item.second);
        tf_tree.Emap.insert(pair<string, edge_descriptor_t>(item.first, e.first));
    }
    
    tf_tree.updateTFOrder();
    tf_tree.updateFtame_trans();


}

} // namespace RobotModel
