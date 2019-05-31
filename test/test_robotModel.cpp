
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

    RobotModel::pJoint_t pjoint = std::make_shared<RobotModel::Joint>();;
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
