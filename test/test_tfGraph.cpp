
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
        pRobot = std::make_shared<RobotModel::RobotModel>(robot_dict);
        pTree = std::make_shared<tf_Graph::TF_Graph>(pRobot->mTf_tree);
    }

    std::shared_ptr<RobotModel::RobotModel>  pRobot;
    std::shared_ptr<tf_Graph::TF_Graph> pTree;
};

TEST_F(TF_GraphTest, isLeafFrame)
{
    EXPECT_EQ(pTree->isLeafFrame("tool0"), true);
    EXPECT_EQ(pTree->isLeafFrame("link_1"), false);
}
