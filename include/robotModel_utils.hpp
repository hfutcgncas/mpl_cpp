#ifndef __robotModel_utils__
#define __robotModel_utils__

#include <iostream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "tf_Graph.hpp"

namespace RobotModel
{

using namespace std;

template <typename T>
vector<T> parseYAMLList(YAML::Node yaml_list)
{
    vector<T> out;
    for (auto it = yaml_list.begin(); it != yaml_list.end(); it++)
    {
        out.push_back((*it).as<T>());
    }
    return out;
}

template <class T>
map<string, T> parseYAMLMap(YAML::Node yaml_map)
{
    map<string, T> M;
    T item;
    for (auto it = yaml_map.begin(); it != yaml_map.end(); ++it)
    {
        // cout<<it->first.as<std::string>()<<endl;
        string name = it->first.as<std::string>();
        item << it->second;
        auto pr = std::make_pair(name, item);
        M.insert(pr);
    }
    return M;
}

template <class T>
inline T norm(vector<T> v)
{
    T rt = 0;
    for (auto item : v)
    {
        rt += (item * item);
    }
    return sqrt(rt);
}

class Joint : public tf_Graph::TF
{
public:
    // string name;
    string child;
    string parent;
    string type;
    vector<float> origin_xyz;
    vector<float> origin_rpy;
    vector<float> axis;

    float limit_effort;
    float limit_velocity;
    float limit_lower;
    float limit_upper;

    double value;

    Eigen::Isometry3d trans_ori;

    void updateTf(double value)
    {
        if (type == "revolute" or type == "continuous")
        {
            Eigen::AngleAxisd dr_v(value, Eigen::Vector3d(axis[0], axis[1], axis[2]));
            Eigen::Isometry3d dr = Eigen::Isometry3d::Identity();
            dr.rotate(dr_v);
            trans = trans_ori * dr;
        }
    }

    void operator<<(const YAML::Node &node)
    {
        type = node["type"].as<string>();
        name = node["name"].as<string>();
        child = node["child"].as<string>();
        parent = node["parent"].as<string>();

        origin_rpy = parseYAMLList<float>(node["origin"]["rpy"]);
        origin_xyz = parseYAMLList<float>(node["origin"]["xyz"]);
        axis = parseYAMLList<float>(node["axis"]);

        value = 0;

        if (node["limit"].IsNull())
        {
            limit_effort = -1;
            limit_velocity = -1;
            limit_lower = 0;
            limit_upper = 0;
        }
        else
        {
            limit_effort = node["limit"]["effort"].as<float>();
            limit_velocity = node["limit"]["velocity"].as<float>();
            limit_lower = node["limit"]["lower"].as<float>();
            limit_upper = node["limit"]["upper"].as<float>();
        }

        // 参考视觉slam14 https://www.cnblogs.com/ChrisCoder/p/10083110.html
        float ori_value = norm<float>(origin_rpy);

        Eigen::AngleAxisd rotation_vector(ori_value, Eigen::Vector3d(origin_rpy[0], origin_rpy[1], origin_rpy[2]));
        trans_ori = Eigen::Isometry3d::Identity();
        trans_ori.rotate(rotation_vector);
        trans_ori.pretranslate(Eigen::Vector3d(origin_xyz[0], origin_xyz[1], origin_xyz[2]));
        trans = trans_ori;

        // std::cout<< "init edge: "<<name<<":"<<std::endl;
        // std::cout<< trans_ori.matrix() <<  std::endl;
        // std::cout<< origin_xyz[0] <<" "<< origin_xyz[1]<<" "<<origin_xyz[2]<<" " <<  std::endl;

        // std::cout<< "xxxxxxxxxxxxxxxxxxxxxxxxx" <<  std::endl;
    }

    Joint &operator=(const Joint &src)
    {
        name = src.name;
        child = src.child;
        parent = src.parent;
        type = src.type;

        origin_rpy.assign(src.origin_rpy.begin(), src.origin_rpy.end());
        origin_xyz.assign(src.origin_xyz.begin(), src.origin_xyz.end());
        axis.assign(src.axis.begin(), src.axis.end());

        limit_effort = src.limit_effort;
        limit_velocity = src.limit_velocity;
        limit_lower = src.limit_lower;
        limit_upper = src.limit_upper;

        trans = src.trans;
        trans_ori = src.trans_ori;
    }

    Joint() : tf_Graph::TF()
    {
        trans_ori = Eigen::Isometry3d::Identity();
    }
    Joint(const Joint &src)
    {
        *this = src;
    }
};

class Link : public tf_Graph::Frame
{
public:
    Link() : tf_Graph::Frame() {}

    // Link(string Name) : name(Name), rt2base(Eigen::Isometry3d::Identity())
    // {
    // }

    // string name;

    void operator<<(const YAML::Node &node)
    {
        name = node["name"].as<string>();
    }

    Link &operator=(const Link &src)
    {
        name = src.name;
        rt2base = src.rt2base;
    }

    Link(const Link &src)
    {
        *this = src;
    }
};

class Joint_Link_pair
{
public:
    string joint_name;
    string link_name;

    void operator<<(const YAML::Node &node)
    {
        vector<string> v = parseYAMLList<string>(node);
        joint_name = v[0];
        link_name = v[1];
    }
};

} // namespace RobotModel

#endif