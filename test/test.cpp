
#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "robotModel.hpp"


TEST(RobotModel,loadYAML_framename)
{
    YAML::Node robot_dict = YAML::LoadFile("/home/liujianran/temp/mpl_cpp/mechmind_yaml_model.yaml");
    RobotModel::RobotModel robot(robot_dict);
    EXPECT_EQ(robot.tf_tree.getFrame_p("tool0")->name,"tool0");
}

int main(int argc,char **argv)
{
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
