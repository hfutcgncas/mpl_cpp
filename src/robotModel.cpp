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

void operator<<(pLink_t &plink, const YAML::Node &node)
{
    plink = std::make_shared<Link>();
    *plink << node; 
}

void operator<<(pJoint_t &pjoint, const YAML::Node &node)
{
    pjoint = std::make_shared<Joint>();
    *pjoint << node; 
}


RobotModel::RobotModel(YAML::Node node)
{
    // ControlableJoints
    mControlableJoints = parseYAMLList<string>(node["ControlableJoints"]);
    mJointMap = parseYAMLMap<pJoint_t>(node["joint_map"]);
    mLinkMap = parseYAMLMap<pLink_t>(node["link_map"]);
    mParentMap = parseYAMLMap<Joint_Link_pair>(node["parent_map"]);
    mChildMap = parseYAMLMap<vector<Joint_Link_pair>>(node["child_map"]);

    build_frame_Tree();
}

void RobotModel::build_frame_Tree()
{
    // Add links
    for (auto item : mLinkMap)
    {
        pFrame_t pframe = dynamic_pointer_cast<Frame>(item.second);
        vertex_descriptor_t v = boost::add_vertex(pframe, mTf_tree.g);
        mTf_tree.Vmap.insert(pair<string, vertex_descriptor_t>(item.first, v));      
    }
    // Add edges
    for (auto item : mJointMap)
    {
        auto v1 = mTf_tree.Vmap[item.second->parent];
        auto v2 = mTf_tree.Vmap[item.second->child];
        pTF_t pframe = dynamic_pointer_cast<TF>(item.second);
        std::pair<edge_descriptor_t, bool> e = boost::add_edge(v1, v2, pframe, mTf_tree.g);
        mTf_tree.Emap.insert(pair<string, edge_descriptor_t>(item.first, e.first));
    }

    mTf_tree.updateTFOrder();
    mTf_tree.updateFtame_trans();
}

bool RobotModel::setJointValue(string jName, double jValue, bool updateTree)
{
    pTF_t ptf = mTf_tree.getTF_p(jName); 
    pJoint_t pj = std::dynamic_pointer_cast<Joint, TF>(ptf) ; 
    assert(pj != NULL);
    pj->updateTf(jValue);

    if(updateTree)
    {
        mTf_tree.updateFtame_trans();
    }
    return true;
}

bool RobotModel::updateJointsValue( map<string, double> jvMap, bool updateTree )
{
    // for(boost::tie(out_i, out_end) = out_edges(v, g);  )
    string name;
    float value;
    for(auto jv : jvMap)
    {
        boost::tie(name, value) = jv;
        setJointValue(name, value, false);
    }

    if(updateTree)
    {
        mTf_tree.updateFtame_trans();
    }
}


vector<string> RobotModel::getControlableJoints()
{
    vector<string> out;
    for(auto item : mJointMap)
    {
        if( item.second->type != "fixed")
        {
            out.push_back(item.first);
        }
    }
    return out;
}


} // namespace RobotModel
