#include "yaml-cpp/yaml.h" //安装yaml-cpp参考google code 主页
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <typeinfo>

using namespace std;
using namespace boost;

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


    Joint &operator = (const Joint& src)
    {
        name = src.name;
        child = src.child;
        parent = src.parent;
        type = src.type;

     
        origin_rpy.assign(src.origin_rpy.begin(), src.origin_rpy.end());
        origin_xyz.assign(src.origin_xyz.begin(), src.origin_xyz.end());
        axis.assign(src.axis.begin(), src.axis.end());
        
        limit_effort =  src.limit_effort;
        limit_velocity =  src.limit_velocity;
        limit_lower  =  src.limit_lower;
        limit_upper  =  src.limit_upper;
    }

    Joint() { }
    Joint(const Joint& src)
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

void operator<<(vector<Joint_Link_pair> &jlSet, const YAML::Node &node)
{
    Joint_Link_pair p;
    // assert(node.IsSequence());

    for (auto it = node.begin(); it != node.end(); ++it)
    {
        p << *it;
        jlSet.push_back(p);
    }
}

struct vertex_link_t
{
    typedef vertex_property_tag kind;
};
struct edge_Joint_t
{
    typedef edge_property_tag kind;
};

typedef property<vertex_link_t, Link> VertexLinkProperty;
typedef property<edge_Joint_t, Joint> EdgeJointProperty;
typedef adjacency_list<listS, vecS, directedS, VertexLinkProperty, Joint> DiGraph;
typedef typename graph_traits<DiGraph>::vertex_descriptor vertex_descriptor_t;
typedef typename graph_traits<DiGraph>::edge_descriptor edge_descriptor_t;

// class TF_Graph{

//     // template <class PropertyTag, class T, class NextProperty = no_property>
//     // struct property;

// }

class RobotModel
{

public:
    // typedef adjacency_list<listS, vecS, directedS, Link*, Joint*> DiGraph;

    RobotModel();
    RobotModel(YAML::Node node)
    {
        // ControlableJoints
        ControlableJoints = parseYAMLList<string>(node["ControlableJoints"]);
        JointMap = parseYAMLMap<Joint>(node["joint_map"]);
        LinkMap = parseYAMLMap<Link>(node["link_map"]);
        ParentMap = parseYAMLMap<Joint_Link_pair>(node["parent_map"]);
        ChildMap = parseYAMLMap<vector<Joint_Link_pair>>(node["child_map"]);

        build_frame_Tree();
    }

    void build_frame_Tree()
    {
        for (auto item : JointMap)
        {
            vertex_descriptor_t v1 = boost::add_vertex(LinkMap[item.second.parent], tf_Graph);
            vertex_descriptor_t v2 = boost::add_vertex(LinkMap[item.second.child], tf_Graph);

            // Add edges
            std::pair<edge_descriptor_t, bool> e = boost::add_edge(v1, v2, tf_Graph);
            tf_Graph[e.first] = item.second;
        }
    }

    // private:

    vector<string> ControlableJoints;
    map<string, Joint> JointMap;
    map<string, Link> LinkMap;
    map<string, Joint_Link_pair> ParentMap;
    map<string, vector<Joint_Link_pair>> ChildMap;

    DiGraph tf_Graph;
};

#include"robotModel.hpp"


int main()
{

    YAML::Node robot_dict = YAML::LoadFile("/home/liujianran/temp/mpl_cpp/mechmind_yaml_model.yaml");

    // std::cout<< robot_dict["name"].as<std::string>() << std::endl;

    // vector<string> as = parseYAMLList<string>(robot_dict["parent_map"]["link_1"]);

    RobotModel::RobotModel robot(robot_dict);

    cout<< robot.tf_Graph.m_vertices.size()<< endl;
    cout<< robot.LinkMap.size()
    // cout<<robot.ChildMap["base_link"][0].link_name<<endl;

    // cout<<"================================="<<endl;
    // for(auto it=as.begin();it!=as.end();++it)
    // {
    //     cout<<*it<<endl;
    // }
    return 0;
}