
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include"robotModel.hpp"


int main()
{

    YAML::Node robot_dict = YAML::LoadFile("/home/liujianran/temp/mpl_cpp/mechmind_yaml_model.yaml");

    RobotModel::RobotModel robot(robot_dict);

    std::cout<< robot.tf_Graph.g.m_vertices.size()<< std::endl;
    std::cout<< robot.LinkMap.size()<< std::endl;
    std::cout<< robot.tf_Graph.getLink_p("link_3")->name<<std::endl ;

    RobotModel::Joint* j =  robot.tf_Graph.getJoint_p("joint_1");
    j->axis[2] = 100;

    std::cout<< robot.tf_Graph.getJoint_p("joint_1")->axis[2]<<std::endl ;


    return 0;
}