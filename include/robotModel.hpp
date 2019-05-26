#ifndef __ROBOTMODEL__
#define __ROBOTMODEL__

#include "robotModel_utils.hpp"
#include "tf_Graph.hpp"

namespace RobotModel
{

using namespace std;


typedef std::shared_ptr<Link>  pLink_t;
typedef std::shared_ptr<Joint>  pJoint_t;

class RobotModel
{

public:
    RobotModel();
    RobotModel(YAML::Node node);
    void build_frame_Tree();

    vector<string> ControlableJoints;
    map<string, Joint> JointMap;
    map<string, Link> LinkMap;
    map<string, Joint_Link_pair> ParentMap;
    map<string, vector<Joint_Link_pair>> ChildMap;


    bool setJointValue(string jName, double jValue, bool updateTree);
    bool updateJointsValue( map<string, double> jvMap, bool updateTree );
    

public:
    tf_Graph::TF_Graph tf_tree;
};

} // namespace RobotModel

#endif