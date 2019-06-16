
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "robotModel.hpp"

#include <boost/graph/graph_traits.hpp>

#include <time.h>


int main()
{
    clock_t start_t, end_t1, end_t2 ;
    double total_t1, total_t2;
   int i;
 
   start_t = clock();

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

    start_t = clock();
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

    end_t1 = clock();
    cout<< robot.isCollision() <<endl;
    end_t2 = clock();

    total_t1 = (double)(end_t1 - start_t) / CLOCKS_PER_SEC;
    total_t2 = (double)(end_t2 - start_t) / CLOCKS_PER_SEC;
   printf("CPU 占用的总时间：%f， %f\n", total_t1, total_t2  );

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