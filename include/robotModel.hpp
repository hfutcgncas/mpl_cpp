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

public:
    RobotModel();
    RobotModel(YAML::Node node);
    void build_frame_Tree();

    vector<string> ControlableJoints;
    map<string, pJoint_t> JointMap;
    map<string, pLink_t> LinkMap;
    map<string, Joint_Link_pair> ParentMap;
    map<string, vector<Joint_Link_pair>> ChildMap;


    bool setJointValue(string jName, double jValue, bool updateTree);
    bool updateJointsValue( map<string, double> jvMap, bool updateTree );
    

public:
    tf_Graph::TF_Graph tf_tree;
};

} // namespace RobotModel

#endif