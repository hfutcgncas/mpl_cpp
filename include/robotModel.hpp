#ifndef __ROBOTMODEL__
#define __ROBOTMODEL__

#include "robotModel_utils.hpp"
#include "tf_Graph.hpp"
#include "robotCollisionDetector.hpp"


namespace RobotModel
{

using namespace std;

void operator<<(pLink_t &plink, const YAML::Node &node);
void operator<<(pJoint_t &pjoint, const YAML::Node &node);
// void operator<<(pLink_geom_t &plink_geom, const YAML::Node &node);
void operator<<(collisionPair_set_t &collisionss_set, const YAML::Node &node);

void operator<<(pLinkGeometry_t &pLinkGeometry, const YAML::Node &node);

class RobotModel
{
    private:
    vector<string> mControlableJoints;
    map<string, pJoint_t> mJointMap;
    map<string, pLink_t> mLinkMap;
    map<string, Joint_Link_pair> mParentMap;
    map<string, vector<Joint_Link_pair>> mChildMap;

    collisionPair_set_t mDisable_collisionss_set;

    set<string> mCollisionLinksSet;
    set<string> mEnvLinksSet;
    set<string> mRobotLinksSet;
    
    CollisionDetector mCollisionDetector;

    public:
    // map<string, pLink_geom_t> mLinkCollisionMap;
    pLinkGeometry_map_t mLinkGeometryMap; 

    RobotModel() {}
    RobotModel(YAML::Node node);

    void loadYAML(YAML::Node node);
    void build_frame_Tree();
    bool setJointValue(string jName, double jValue, bool updateTree);
    bool updateJointsValue(map<string, double> jvMap, bool updateTree);

    vector<string> getControlableJoints();
    std::string getRootName();

    bool addLink(const pLink_t plink, string parentName, pJoint_t ptf);
    bool rmLink(const string linkname);
    bool rmLink_recursive(const string linkname);
    pLink_t getLink_p(string name);
    pJoint_t getJoint_p(string name);
    // 当name不在图里时，返回NULL而非报错。 这样比较耗时
    pLink_t getLink_p_safe(string name);
    pJoint_t getJoint_p_safe(string name);

    bool ChangeParentLink(string targetName, string newParentName);



    // TO DO

    bool getJoint();
    bool SwitchLinkParent();

    public:
    tf_Graph::TF_Graph mTf_tree;

    bool reset()
    {
        mCollisionLinksSet.clear();
        for(auto item : mLinkGeometryMap)
        {
            mCollisionLinksSet.insert(item.first);
        }
        mEnvLinksSet = getFixedChildLink( getRootName() );
        mRobotLinksSet.clear();
        set_intersection(   mCollisionLinksSet.begin(), 
                            mCollisionLinksSet.end(),
                            mEnvLinksSet.begin(), 
                            mEnvLinksSet.end(), 
                            inserter(mRobotLinksSet, mRobotLinksSet.begin()) );   
    }

    std::set<std::string> getFixedChildLink(std::string baselink)
    {
        std::set<std::string> rt;
        if (mTf_tree.Vmap.find(baselink) == mTf_tree.Vmap.end())
        {
            return rt;
        }
        else
        { 
            set<pair<tf_Graph::pFrame_t, tf_Graph::pTF_t>> childset = mTf_tree.getChildVE(baselink);
            for(auto childPair : childset)
            {
                tf_Graph::pFrame_t pf;
                tf_Graph::pTF_t ptf;
                tie(pf, ptf) = childPair;
                pJoint_t pj = dynamic_pointer_cast<Joint>(ptf);
                if(pj->type == "fixed")
                {
                    rt.insert(pf->name);
                    for(auto link: getFixedChildLink(pf->name))
                    {
                        rt.insert(link);
                    }
                }
            }
            return rt;
        }
    }

    bool isCollision()
    {
        return mCollisionDetector.checkCollision( mTf_tree.tfMap );
    }

    bool isCollision( map<string, double> jvMap )
    {
        updateJointsValue(jvMap, true);
        return mCollisionDetector.checkCollision( mTf_tree.tfMap );
    }

};

} // namespace RobotModel

#endif