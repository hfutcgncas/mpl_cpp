
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
        pRobot = std::make_shared<RobotModel::RobotModel>(robot_dict);
    }

    std::shared_ptr<RobotModel::RobotModel> pRobot;
};

TEST_F(RobotModelTest, loadYAML_framename)
{
    EXPECT_EQ(pRobot->mTf_tree.getFrame_p("tool0")->name, "tool0");
}

TEST_F(RobotModelTest, getControlableJoints)
{
    std::vector<std::string> cj = pRobot->getControlableJoints();
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
    EXPECT_EQ(pRobot->getRootName(), "base_link");
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
    EXPECT_TRUE(pRobot->addLink(plink, parentlink, pjoint));

    // ------------------------------------
    // pjoint->parent = "link_x";
    // EXCECT ( pRobot->addLink(plink, parentlink, pjoint)  );
    
}

TEST_F(RobotModelTest, getLink_p)
{
    EXPECT_EQ(pRobot->getLink_p("tool0")->name, "tool0");
}

TEST_F(RobotModelTest, rmLink)
{
    EXPECT_FALSE(pRobot->rmLink("link_1"));
    EXPECT_TRUE(pRobot->rmLink("tool0"));
    EXPECT_EQ(pRobot->getLink_p_safe("tool0"), nullptr);
}

TEST_F(RobotModelTest, rmLink_recursive)
{
    EXPECT_TRUE(pRobot->rmLink_recursive("link_1"));
    EXPECT_EQ(pRobot->getLink_p_safe("link_1"), nullptr);
    EXPECT_EQ(pRobot->getLink_p_safe("tool0"), nullptr);
}
