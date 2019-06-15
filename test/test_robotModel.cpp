
#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "robotModel.hpp"

using namespace std;

class RobotModelTest : public testing::Test
{
protected:
    virtual void SetUp() override
    {
        YAML::Node robot_dict = YAML::LoadFile("/home/liujianran/temp/mpl_cpp/mechmind_yaml_model.yaml");
        Robot.loadYAML(robot_dict);
    }

    RobotModel::RobotModel Robot;
};

TEST_F(RobotModelTest, loadYAML_framename)
{
    EXPECT_EQ(Robot.mTf_tree.getFrame_p("tool0")->name, "tool0");
}

TEST_F(RobotModelTest, getControlableJoints)
{
    std::vector<std::string> cj = Robot.getControlableJoints();
    std::vector<std::string> gt;
    gt.push_back("joint_1");
    gt.push_back("joint_2");
    gt.push_back("joint_3");
    gt.push_back("joint_4");
    gt.push_back("joint_5");
    gt.push_back("joint_6");

    EXPECT_EQ(cj, gt);
}

TEST_F(RobotModelTest, getRootName)
{
    EXPECT_EQ(Robot.getRootName(), "base_link");
}

TEST_F(RobotModelTest, addLink)
{
    string parentlink = "link_1";

    RobotModel::pLink_t plink = std::make_shared<RobotModel::Link>();
    plink->name = "new_link";

    RobotModel::pJoint_t pjoint = std::make_shared<RobotModel::Joint>();
    ;
    pjoint->name = "new_joint";
    pjoint->parent = parentlink;
    pjoint->child = plink->name;
    EXPECT_TRUE(Robot.addLink(plink, parentlink, pjoint));

    // ------------------------------------
    // pjoint->parent = "link_x";
    // EXCECT ( Robot.addLink(plink, parentlink, pjoint)  );
}

TEST_F(RobotModelTest, getLink_p)
{
    EXPECT_EQ(Robot.getLink_p("tool0")->name, "tool0");
}

TEST_F(RobotModelTest, rmLink)
{
    // EXPECT_TRUE(Robot.rmLink("tool0")); 
    EXPECT_FALSE(Robot.rmLink("link_1"));
    EXPECT_TRUE(Robot.rmLink("tool0"));
    EXPECT_EQ(Robot.getLink_p_safe("tool0"), nullptr);
}

TEST_F(RobotModelTest, rmLink_recursive)
{
    EXPECT_TRUE(Robot.rmLink_recursive("link_1"));
    EXPECT_EQ(Robot.getLink_p_safe("link_1"), nullptr);
    EXPECT_EQ(Robot.getLink_p_safe("tool0"), nullptr);
}

// sucker 有时会碰撞失效，不知道是否是fcl新版本的问题，后面再观察 
// TEST_F(RobotModelTest, collision)
// {
//     // Robot.
//     std::map<std::string, double> jvm;
    
//     Robot.updateJointsValue(jvm, true);
    
//     EXPECT_TRUE(Robot.isCollision("link_1", "link_2"));
//     EXPECT_FALSE(Robot.isCollision("link_1", "link_3"));
//     EXPECT_FALSE(Robot.isCollision("link_2", "sucker"));
//     EXPECT_TRUE(Robot.isCollision("link_5", "link_6"));

    
//     jvm["joint_1"] = 0;
//     jvm["joint_2"] = 0.31;
//     jvm["joint_3"] = 1.07;
//     jvm["joint_4"] = 0;
//     jvm["joint_5"] = 1.08;
//     jvm["joint_6"] = 0;
  

//     Robot.updateJointsValue(jvm, true);
    
//     EXPECT_TRUE(Robot.isCollision("link_1", "sucker"));
//     // // EXPECT_TRUE(Robot.isCollision("sucker", "link_2"));
    
// }

TEST_F(RobotModelTest, collision)
{
    std::map<std::string, double> jvm;
    jvm["joint_1"] = 0;
    jvm["joint_2"] =  1.27;
    jvm["joint_3"] = 1.22;
    jvm["joint_4"] = 0.18;
    jvm["joint_5"] = 1.22;
    jvm["joint_6"] = 0.00;
    Robot.updateJointsValue(jvm, true);

    

    EXPECT_TRUE(Robot.isCollision(jvm));
    // // EXPECT_TRUE(Robot.isCollision("sucker", "link_2"));
    
}

TEST_F(RobotModelTest, getFixedChildLink)
{
    set<string> t{"base", "foundation", "foundation_box", "led", "mechmind_camera", "table"};
    set<string> s = Robot.getFixedChildLink( Robot.getRootName() );
    EXPECT_EQ( s, t );
    
}


// no good
TEST_F(RobotModelTest, ChangeParentLink)
{
    EXPECT_TRUE(Robot.ChangeParentLink("tool0", "link_1"));
}
