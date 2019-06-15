
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
  
    // std::cout<< robot.mTf_tree.getFrame_p("tool0")->rt2base.matrix()<< std::endl;

    // std::map<std::string, double> jvm;
    // jvm.insert( std::pair<std::string, double>("joint_1", 1));
    // jvm.insert( std::pair<std::string, double>("joint_3", -2.02));
    // jvm.insert( std::pair<std::string, double>("joint_5", 2.01));

    // robot.updateJointsValue(jvm, true);
    // std::cout<< robot.mTf_tree.getFrame_p("tool0")->rt2base.matrix()<< std::endl;

    // for(auto i : robot.getControlableJoints())
    // {
    //     std::cout<< i << std::endl;
    // }

    // std::cout<< robot.getRootName()<< std::endl;

    // robot.mTf_tree.rmFrame_recursive("link_6");


    std::map<std::string, double> jvm;
    jvm.insert( std::pair<std::string, double>("joint_1", 1));
    jvm.insert( std::pair<std::string, double>("joint_3", -2.02));
    jvm.insert( std::pair<std::string, double>("joint_5", 2.01));

    jvm["joint_1"] = 0;
    jvm["joint_2"] = 1.27;
    jvm["joint_3"] = 1.22;
    jvm["joint_4"] = 0.18;
    jvm["joint_5"] = 1.22;
    jvm["joint_6"] = 0;

    robot.updateJointsValue(jvm, true);

    cout<< robot.isCollision() <<endl;

    jvm["joint_1"] = 0;
    jvm["joint_2"] = 0;
    jvm["joint_3"] = 0;
    jvm["joint_4"] = 0;
    jvm["joint_5"] = 0;
    jvm["joint_6"] = 0;


    robot.updateJointsValue(jvm, true);
    cout<< robot.isCollision() <<endl;

    jvm["joint_1"] = 0;
    jvm["joint_2"] = 1.27;
    jvm["joint_3"] = 0;
    jvm["joint_4"] = 0;
    jvm["joint_5"] = 0;
    jvm["joint_6"] = 0;


    robot.updateJointsValue(jvm, true);
    cout<< robot.isCollision() <<endl;

    
    return 0;
}