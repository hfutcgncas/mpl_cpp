#ifndef __robotModel_utils__
#define __robotModel_utils__

#include <iostream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "tf_Graph.hpp"

// // fcl
// #include "fcl/fcl.h"

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

        if(node["axis"].IsNull())
        {
            axis.push_back(0);
            axis.push_back(0);
            axis.push_back(1);
        }
        else
        {
            axis = parseYAMLList<float>(node["axis"]);
        }
        

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

        // // 参考视觉slam14 https://www.cnblogs.com/ChrisCoder/p/10083110.html
        // float ori_value = norm<float>(origin_rpy);

        // Eigen::AngleAxisd rotation_vector(ori_value, Eigen::Vector3d(origin_rpy[0], origin_rpy[1], origin_rpy[2]));
        // trans_ori = Eigen::Isometry3d::Identity();
        // trans_ori.rotate(rotation_vector);
        // trans_ori.pretranslate(Eigen::Vector3d(origin_xyz[0], origin_xyz[1], origin_xyz[2]));
        // trans = trans_ori;

        // rpy转旋转矩阵，参考 https://blog.csdn.net/weicao1990/article/details/86148828
        //3.0 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)
        Eigen::Vector3d ea(origin_rpy[0], origin_rpy[1], origin_rpy[2]);
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix =   Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                            Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                            Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
        trans_ori = Eigen::Isometry3d::Identity();
        trans_ori.rotate(rotation_matrix);
        trans_ori.pretranslate(Eigen::Vector3d(origin_xyz[0], origin_xyz[1], origin_xyz[2]));
        trans = trans_ori;


        // std::cout<< "init edge: "<<name<<":"<<std::endl;
        // std::cout<< trans_ori.matrix() <<  std::endl;
        // // std::cout<< origin_xyz[0] <<" "<< origin_xyz[1]<<" "<<origin_xyz[2]<<" " <<  std::endl;

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
    Link() : tf_Graph::Frame()
    {
    }

    void operator<<(const YAML::Node &node)
    {
        name = node["name"].as<std::string>();
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

// class Link_geom
// {
//     public:
//         std::shared_ptr<fcl::CollisionObject<double>> obj;
//         fcl::Transform3d tf_base; //  fcl::Transform3d <=> Eigen::Isometry3d

//         Link_geom()
//         {
//             obj = nullptr;
//             tf_base = Eigen::Isometry3d::Identity();
//         }

//         void operator<<(const YAML::Node &node)
//         {
//             // collision obj
//             std::string type = node["geom"]["type"].as<std::string>();
//             if (type == "Box")
//             {
//                 std::vector<double> value = parseYAMLList<double>(node["geom"]["value"]);
//                 std::shared_ptr<fcl::Box<double>> obj_geom = std::make_shared<fcl::Box<double>>(value[0], value[1], value[2]);
//                 obj = std::make_shared<fcl::CollisionObject<double>>(obj_geom);
//             }
//             else if (type == "Cylinder")
//             {
//                 std::vector<double> value = parseYAMLList<double>(node["geom"]["value"]);
//                 std::shared_ptr<fcl::Cylinder<double>> obj_geom = std::make_shared<fcl::Cylinder<double>>(value[0], value[1]);
//                 obj = std::make_shared<fcl::CollisionObject<double>>(obj_geom);
//             }
//             else if (type == "Sphere")
//             {
//                 double value = node["geom"]["value"].as<double>();
//                 std::shared_ptr<fcl::Sphere<double>> obj_geom = std::make_shared<fcl::Sphere<double>>(value);
//                 obj = std::make_shared<fcl::CollisionObject<double>>(obj_geom);
//             }
//             else if (type == "BVHModel")
//             {
//                 /* code */
//                 vector<vector<fcl::Vector3d>> value;
//                 for (auto triangleNode : node["geom"]["value"])
//                 {
//                     vector<fcl::Vector3d> triangle;
//                     for (auto vNode : triangleNode)
//                     {
//                         fcl::Vector3d v;
//                         v[0] = vNode[0].as<double>();
//                         v[1] = vNode[1].as<double>();
//                         v[2] = vNode[2].as<double>();
//                         triangle.push_back(v);
//                     }
//                     value.push_back(triangle);
//                 }
//                 int cntv = value.size();
//                 // 这里用推荐的OBBRSS会出现问题，某些角度下碰撞检测不准
//                 std::shared_ptr<fcl::BVHModel<fcl::AABBd>> obj_geom = std::make_shared<fcl::BVHModel<fcl::AABBd>>();
//                 obj_geom->beginModel(cntv * 3, cntv);
//                 for (int i = 0; i < cntv; i++)
//                 {
//                     obj_geom->addTriangle(value[i][0], value[i][1], value[i][2]);
//                 }
//                 obj_geom->endModel();
//                 obj = std::make_shared<fcl::CollisionObject<double>>(obj_geom);
//             }
//             else
//             {
//                 std::cout << "Not vailed geom type" << std::endl;
//                 obj = nullptr;
//             }

//             // tf.

//             Eigen::Matrix4d tf_mat = tf_base.matrix();
//             // Eigen::Vector3d translation;
//             for (int i = 0; i < 4; i++)
//             {
//                 for (int j = 0; j < 4; j++)
//                 {
//                     tf_mat(i, j) = node["tf"][i][j].as<double>();
//                 }
//                 // translation[i] = tfNode[i][3].as<double>();
//             }

//             // 参考视觉slam14 https://www.cnblogs.com/ChrisCoder/p/10083110.html
//             // float ori_value = norm<float>(origin_rpy);

//             // Eigen::AngleAxisd r;
//             // r.fromRotationMatrix(tf_R_mat);
//             // tf = Eigen::Isometry3d::Identity();
//             // tf.rotate(r);
//             // tf.pretranslate(translation);
//         }

//         bool setTF(Eigen::Isometry3d tf_link)
//         {
//             if (obj != nullptr)
//             {
//                 // obj->setRotation
//                 Eigen::Isometry3d t1 = tf_link * tf_base;

//                 obj->setTranslation(t1.translation());
//                 obj->setRotation(t1.rotation());

//                 // obj->setTransform( tf_link*tf_base );
//                 // obj->setTransform( tf_link);
//                 return true;
//             }
//             else
//             {
//                 return false;
//             }
//         }
// };

class Link_geometry
{
    public:
    // string type;
    YAML::Node geometryNode;
    Eigen::Isometry3d tf_base; //  fcl::Transform3d <=> Eigen::Isometry3d

    Link_geometry()
    {
        tf_base = Eigen::Isometry3d::Identity();
    }

    void operator<<(const YAML::Node &node)
    {
            // tf
            Eigen::Matrix4d tf_mat;
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    tf_mat(i, j) = node["tf"][i][j].as<double>();
                    
            tf_base.matrix() = tf_mat;
            // cout<<"aa"<<endl<< tf_base.matrix()<<endl;
            // geom
            geometryNode = node["geom"];
    }
};


typedef std::shared_ptr<Link> pLink_t;
typedef std::shared_ptr<Joint> pJoint_t;
// typedef std::shared_ptr<Link_geom> pLink_geom_t;

typedef std::set<string> collisionPair_t;
typedef std::set<collisionPair_t> collisionPair_set_t;

typedef std::shared_ptr<Link_geometry> pLinkGeometry_t;
typedef std::map<string, pLinkGeometry_t> pLinkGeometry_map_t;

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