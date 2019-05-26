
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "robotModel.hpp"

#include <boost/graph/graph_traits.hpp>

int main()
{

    YAML::Node robot_dict = YAML::LoadFile("/home/liujianran/temp/mpl_cpp/mechmind_yaml_model.yaml");

    RobotModel::RobotModel robot(robot_dict);


    std::cout<< robot.tf_tree.getFrame_p("tool0")->rt2base.matrix()<< std::endl;

    std::map<std::string, double> jvm;
    jvm.insert( std::pair<std::string, double>("joint_1", 1));
    jvm.insert( std::pair<std::string, double>("joint_3", -2.02));
    jvm.insert( std::pair<std::string, double>("joint_5", 2.01));
    robot.updateJointsValue(jvm, true);
    std::cout<< robot.tf_tree.getFrame_p("tool0")->rt2base.matrix()<< std::endl;

    

    return 0;
}