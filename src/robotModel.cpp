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

// << for Joint_Link_pair list
void operator<<(vector<Joint_Link_pair> &jlSet, const YAML::Node &node)
{
    Joint_Link_pair p;
    for (auto it = node.begin(); it != node.end(); ++it)
    {
        p << *it;
        jlSet.push_back(p);
    }
}

// << for pLink_t
void operator<<(pLink_t &plink, const YAML::Node &node)
{
    plink = std::make_shared<Link>();
    *plink << node;
}

// << for pJoint_t
void operator<<(pJoint_t &pjoint, const YAML::Node &node)
{
    pjoint = std::make_shared<Joint>();
    *pjoint << node;
}

// << for pLinkGeometry_t
void operator<<(pLinkGeometry_t &pLinkGeometry, const YAML::Node &node)
{
    pLinkGeometry = std::make_shared<Link_geometry>();
    *pLinkGeometry << node;
}

void operator<<(collisionPair_set_t &collisionss_set, const YAML::Node &node)
{
    for (auto pair : node)
    {
        collisionPair_t cp;
        for (auto link : pair)
        {
            cp.insert(link.as<string>());
        }
        collisionss_set.insert(cp);
    }
}

void RobotModel::loadYAML(YAML::Node &node)
{

    // ControlableJoints
    mControlableJoints = parseYAMLList<string>(node["ControlableJoints"]);
    mJointMap = parseYAMLMap<pJoint_t>(node["joint_map"]);
    mLinkMap = parseYAMLMap<pLink_t>(node["link_map"]);
    mParentMap = parseYAMLMap<Joint_Link_pair>(node["parent_map"]);
    mChildMap = parseYAMLMap<vector<Joint_Link_pair>>(node["child_map"]);

    mLinkGeometryMap = parseYAMLMap<pLinkGeometry_t>(node["obj_geom_dict"]);

    

    mDisable_collisionss_set << node["disable_collisionss_set"];

    build_frame_Tree();
}

RobotModel::RobotModel(YAML::Node node)
{
    loadYAML(node);
    init();
}

void RobotModel::init()
{
    set<string> collisionLinks;
    set<string> robotLinks;
    set<string> envLinks;

    for (auto item : mLinkGeometryMap)
    {
        collisionLinks.insert(item.first);
    }
    envLinks = getFixedChildLink(getRootName());
    envLinks.insert(getRootName());

    set_difference(
        collisionLinks.begin(),
        collisionLinks.end(),
        envLinks.begin(),
        envLinks.end(),
        inserter(robotLinks, robotLinks.begin()));

    mCollisionDetector.setup(mLinkGeometryMap, envLinks, robotLinks, mDisable_collisionss_set);

    map<string, double> jvMap;
    updateJointsValue(jvMap, true); //init pose for envs;
    mTf_tree.updateTfMap(); // update mTf_tree.tfMap
    mCollisionDetector.updateEnvPose(mTf_tree.tfMap);
    mCollisionDetector.updateRobotPose(mTf_tree.tfMap);
   
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
        pTF_t ptf = dynamic_pointer_cast<TF>(item.second);
        std::pair<edge_descriptor_t, bool> e = boost::add_edge(v1, v2, ptf, mTf_tree.g);
        mTf_tree.Emap.insert(pair<string, edge_descriptor_t>(item.first, e.first));
    }

    mTf_tree.updateVEmap();
    mTf_tree.updateTFOrder();
    mTf_tree.updateFrame_trans();
}

bool RobotModel::setJointValue(string jName, double jValue, bool updateTree)
{
    pTF_t ptf = mTf_tree.getTF_p(jName);
    pJoint_t pj = std::dynamic_pointer_cast<Joint, TF>(ptf);
    assert(pj != NULL);
    pj->updateTf(jValue);

    if (updateTree)
    {
        mTf_tree.updateFrame_trans();
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
        mTf_tree.updateFrame_trans();

        // for(auto clink: mLinkCollisionMap)
        // {
        //     string name;
        //     pLink_geom_t plg;
        //     tie(name, plg) = clink;
        //     pLink_t pl = getLink_p(name);
        //     plg->setTF(pl->rt2base);
        //     // cout<<"name: "<<name<<endl;
        //     // cout<<"tf: "<<plg-><<endl;
        // }
    }
}

vector<string> RobotModel::getControlableJoints()
{
    vector<string> out;
    for (auto item : mJointMap)
    {
        if (item.second->type != "fixed")
        {
            out.push_back(item.first);
        }
    }
    return out;
}

std::string RobotModel::getRootName()
{
    return mTf_tree.getRootFrameName();
}

// 增
bool RobotModel::addLink(const pLink_t plink, const string parentName, const pJoint_t pjoint)
{
    assert(pjoint->parent == parentName);
    assert(pjoint->child == plink->name);

    bool success = mTf_tree.add_Frame(dynamic_pointer_cast<Frame>(plink), parentName, dynamic_pointer_cast<TF>(pjoint));
    mTf_tree.updateTFOrder();
    mTf_tree.updateFrame_trans();
    return success;
}

// 删
bool RobotModel::rmLink(const string linkName)
{
    if (mTf_tree.isFrameInTree(linkName))
    {
        mTf_tree.rmFrame(linkName);
    }
    else
    {
        return false;
    }
}

bool RobotModel::rmLink_recursive(const string linkName)
{
    if (mTf_tree.isFrameInTree(linkName))
    {
        mTf_tree.rmFrame_recursive(linkName);
    }
    else
    {
        return false;
    }
}

// 查
pLink_t RobotModel::getLink_p(string name)
{
    return dynamic_pointer_cast<Link>(mTf_tree.getFrame_p(name));
}

pJoint_t RobotModel::getJoint_p(string name)
{
    return dynamic_pointer_cast<Joint>(mTf_tree.getTF_p(name));
}

pLink_t RobotModel::getLink_p_safe(string name)
{
    return dynamic_pointer_cast<Link>(mTf_tree.getFrame_p_safe(name));
}
pJoint_t RobotModel::getJoint_p_safe(string name)
{
    return dynamic_pointer_cast<Joint>(mTf_tree.getTF_p_safe(name));
}

bool RobotModel::ChangeParentLink(string targetName, string newParentName)
{
    mTf_tree.ChangeParent(targetName, newParentName);
    pTF_t ptf;
    pFrame_t pf;
    tie(pf, ptf) = mTf_tree.getParentVE(targetName);
    pJoint_t pj = dynamic_pointer_cast<Joint>(ptf);
    pj->trans_ori = pj->trans;
    pj->type = "fixed";
    return true;
}

// bool RobotModel::isCollision(string link1, string link2 )
// {
//     auto iplg1 = mLinkCollisionMap.find(link1);
//     auto iplg2 = mLinkCollisionMap.find(link2);
//     if( iplg1 == mLinkCollisionMap.end() || iplg2 == mLinkCollisionMap.end() )
//     {
//         return false;
//     }

//     pLink_geom_t plg1 = iplg1->second;
//     pLink_geom_t plg2 = iplg2->second;

//             // pLink_t pl1 = getLink_p(link1);
//     //         plg1->setTF(pl1->rt2base);

//     // pLink_t pl2 = getLink_p(link2);
//     //         plg2->setTF(pl2->rt2base);

//     // set the collision request structure, here we just use the default setting
//     fcl::CollisionRequest<double> request;
//     // result will be returned via the collision result structure
//     fcl::CollisionResult<double> result;
//     // perform collision test
//     result.clear();

//     fcl::CollisionObjectd * o1, *o2;
//     o1 = plg1->obj.get();
//     o2 = plg2->obj.get();
//     // o1->setTransform(pl1->rt2base*plg1->tf_base);
//     // o2->setTransform(pl2->rt2base*plg2->tf_base);
//     fcl::collide<double>(o1, o2, request, result);

//     // cout<<"here2"<<endl;
//     return result.isCollision();
// }

} // namespace RobotModel
