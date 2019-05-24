#ifndef __robotModel_utils__
#define __robotModel_utils__

#include <iostream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

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

class Joint
{
public:
    string name;
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

    void operator<<(const YAML::Node &node)
    {
        type = node["type"].as<string>();
        name = node["name"].as<string>();
        child = node["child"].as<string>();
        parent = node["parent"].as<string>();

        origin_rpy = parseYAMLList<float>(node["origin"]["rpy"]);
        origin_xyz = parseYAMLList<float>(node["origin"]["xyz"]);
        axis = parseYAMLList<float>(node["axis"]);

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
    }

    Joint() {}
    Joint(const Joint &src)
    {
        *this = src;
    }
};

class Link
{
public:
    Link() {}

    Link(string Name) : name(Name)
    {
    }

    string name;

    void operator<<(const YAML::Node &node)
    {
        name = node["name"].as<string>();
    }

    Link &operator=(const Link &src)
    {
        name = src.name;
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