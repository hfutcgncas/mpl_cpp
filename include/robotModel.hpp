#ifndef __ROBOTMODEL__
#define __ROBOTMODEL__

#include "robotModel_utils.hpp"
#include "tf_Graph.hpp"

namespace RobotModel
{

using namespace std;

void operator<<(pLink_t &plink, const YAML::Node &node);
void operator<<(pJoint_t &pjoint, const YAML::Node &node);

class RobotModel
{
private:
    vector<string> mControlableJoints;
    map<string, pJoint_t> mJointMap;
    map<string, pLink_t> mLinkMap;
    map<string, Joint_Link_pair> mParentMap;
    map<string, vector<Joint_Link_pair>> mChildMap;

public:
    RobotModel();
    RobotModel(YAML::Node node);

    void build_frame_Tree();
    bool setJointValue(string jName, double jValue, bool updateTree);
    bool updateJointsValue(map<string, double> jvMap, bool updateTree);

    vector<string> getControlableJoints();
    std::string getRootName();

    // TO DO
    bool addLink();
    bool SwitchLinkParent();

  

public:
    tf_Graph::TF_Graph mTf_tree;
};

} // namespace RobotModel

#endif