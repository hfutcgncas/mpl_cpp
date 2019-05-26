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
    //  template<class T, class U>
    //     shared_ptr<T> dynamic_pointer_cast(shared_ptr<U> const & r); // never throws

    for (auto item : LinkMap)
    {
        // pFrame_t pframe = dynamic_pointer_cast<pFrame_t>(shared_ptr<Link>(const & item.second));
        pLink_t plink(new Link());
        *plink = item.second;
        pFrame_t pframe = dynamic_pointer_cast<Frame>(plink);
        vertex_descriptor_t v = boost::add_vertex(pframe, tf_tree.g);
        tf_tree.Vmap.insert(pair<string, vertex_descriptor_t>(item.first, v));
    }

    for (auto item : JointMap)
    {
        auto v1 = tf_tree.Vmap[item.second.parent];
        auto v2 = tf_tree.Vmap[item.second.child];

        // Add edges

        pJoint_t pJoint(new Joint(item.second));
        pTF_t pframe = dynamic_pointer_cast<TF>(pJoint);
        std::pair<edge_descriptor_t, bool> e = boost::add_edge(v1, v2, tf_tree.g);
        tf_tree.g[e.first] = pframe;
        tf_tree.Emap.insert(pair<string, edge_descriptor_t>(item.first, e.first));
    }

    tf_tree.updateTFOrder();
    tf_tree.updateFtame_trans();
}

bool RobotModel::setJointValue(string jName, double jValue, bool updateTree)
{
    pTF_t ptf = tf_tree.getTF_p(jName);
    pJoint_t pj = std::dynamic_pointer_cast<Joint, TF>(ptf);
    assert(pj != NULL);
    pj->updateTf(jValue);

    if (updateTree)
    {
        tf_tree.updateFtame_trans();
    }
    return true;
}

bool RobotModel::updateJointsValue(map<string, double> jvMap, bool updateTree)
{
    // for(boost::tie(out_i, out_end) = out_edges(v, g);  )
    string name;
    float value;
    for (auto jv : jvMap)
    {
        boost::tie(name, value) = jv;
        setJointValue(name, value, false);
    }

    if (updateTree)
    {
        tf_tree.updateFtame_trans();
    }
}

} // namespace RobotModel
