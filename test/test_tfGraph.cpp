
#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "robotModel.hpp"
#include "tf_Graph.hpp"


class TF_GraphTest : public testing::Test
{
protected:
    virtual void SetUp() override
    {
        YAML::Node robot_dict = YAML::LoadFile("/home/liujianran/temp/mpl_cpp/mechmind_yaml_model.yaml");
        Robot.loadYAML(robot_dict);
        pTree = std::make_shared<tf_Graph::TF_Graph>(Robot.mTf_tree);
    }

    RobotModel::RobotModel Robot;
    std::shared_ptr<tf_Graph::TF_Graph> pTree;
};

TEST_F(TF_GraphTest, isLeafFrame)
{
    EXPECT_TRUE(pTree->isLeafFrame("tool0"));
    EXPECT_FALSE(pTree->isLeafFrame("link_1"));
}

TEST_F(TF_GraphTest, rmFrame)
{
    EXPECT_FALSE(pTree->rmFrame("link_1"));
    EXPECT_TRUE(pTree->rmFrame("tool0"));

    bool flag = false;
    for(auto item: pTree->Vmap )
    {
        EXPECT_NE(item.first, "tool0");
        if(item.first == "link_1")
        {
            flag = true;
        }
    }
    EXPECT_TRUE(flag);

}

TEST_F(TF_GraphTest, rmFrame_recursive)
{
    EXPECT_TRUE(pTree->rmFrame_recursive("link_1"));
   
    bool flag = false;
    for(auto item: pTree->Vmap )
    {
        EXPECT_NE(item.first, "link_6");
        EXPECT_NE(item.first, "link_2");
        if(item.first == "base_link")
        {
            flag = true;
        }
    }
    EXPECT_TRUE(flag);
}

TEST_F(TF_GraphTest, add_Frame1)
{
    tf_Graph::pFrame_t pf = std::make_shared<tf_Graph::Frame>();
    pf->name = "tool1";
    pf->rt2base = Eigen::Isometry3d::Identity();

    tf_Graph::pTF_t ptf = std::make_shared<tf_Graph::TF>();
    ptf->name = "joint_add";
    ptf->parent = "link_2";
    ptf->child = pf->name;
    ptf->trans = Eigen::Isometry3d::Identity();

    EXPECT_TRUE(pTree->add_Frame(pf, "link_2", ptf ));

}


TEST_F(TF_GraphTest, add_Frame2)
{
    EXPECT_TRUE(pTree->add_Frame("too1", "link_2", Eigen::Isometry3d::Identity() ));
    EXPECT_TRUE(pTree->add_Frame("too2", "link_22", Eigen::Isometry3d::Identity() ));
}

TEST_F(TF_GraphTest, getParentVE)
{   
    tf_Graph::pFrame_t pf ;
    tf_Graph::pTF_t ptf ;

    boost::tie(pf, ptf) =  pTree->getParentVE("link_6");
    EXPECT_TRUE(pf->name == "link_5");
    EXPECT_TRUE(ptf->name == "joint_6");

    boost::tie(pf, ptf) =  pTree->getParentVE("base_link");
    EXPECT_TRUE(pf->name == "base_link");
    EXPECT_TRUE(ptf  == nullptr);

    boost::tie(pf, ptf) =  pTree->getParentVE("tool0");
    EXPECT_TRUE(pf->name == "link_6");
}