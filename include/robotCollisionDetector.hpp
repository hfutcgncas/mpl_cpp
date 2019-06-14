#pragma once

#include "yaml-cpp/yaml.h"
#include <vector>
#include "robotModel_utils.hpp"
#include "fcl/fcl.h"

// #define DEBUG 

using namespace std;
using namespace RobotModel;



class CollisionLink
{
    public:
    std::shared_ptr<fcl::CollisionObject<double>> collisionObj;
    // fcl::Transform3d tf_base; //  fcl::Transform3d <=> Eigen::Isometry3d
    Eigen::Isometry3d tf_base;

    CollisionLink()
    {   
        collisionObj.reset();
        tf_base = Eigen::Isometry3d::Identity();
    }

    void loadGeometry(const pLinkGeometry_t pLinkGeometry)
    {
        std::string type = pLinkGeometry->geometryNode["type"].as<string>();

        if (type == "Box")
        {
            std::vector<double> value = parseYAMLList<double>(pLinkGeometry->geometryNode["value"]);
            std::shared_ptr<fcl::Box<double>> obj_geom = std::make_shared<fcl::Box<double>>(value[0], value[1], value[2]);
            collisionObj = std::make_shared<fcl::CollisionObject<double>>(obj_geom);
        }
        else if (type == "Cylinder")
        {
            std::vector<double> value = parseYAMLList<double>(pLinkGeometry->geometryNode["value"]);
            std::shared_ptr<fcl::Cylinder<double>> obj_geom = std::make_shared<fcl::Cylinder<double>>(value[0], value[1]);
            collisionObj = std::make_shared<fcl::CollisionObject<double>>(obj_geom);
        }
        else if (type == "Sphere")
        {
            double value = pLinkGeometry->geometryNode["value"].as<double>();
            std::shared_ptr<fcl::Sphere<double>> obj_geom = std::make_shared<fcl::Sphere<double>>(value);
            collisionObj = std::make_shared<fcl::CollisionObject<double>>(obj_geom);
        }
        else if (type == "BVHModel")
        {
            vector<vector<fcl::Vector3d>> value;
            for (auto triangleNode : pLinkGeometry->geometryNode["value"])
            {
                vector<fcl::Vector3d> triangle;
                for (auto vNode : triangleNode)
                {
                    fcl::Vector3d v;
                    v[0] = vNode[0].as<double>();
                    v[1] = vNode[1].as<double>();
                    v[2] = vNode[2].as<double>();
                    triangle.push_back(v);
                }
                value.push_back(triangle);
            }
            int cntv = value.size();
            // 这里用推荐的OBBRSS会出现问题，某些角度下碰撞检测不准
            std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> obj_geom = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
            obj_geom->beginModel(cntv * 3, cntv);
            for (int i = 0; i < cntv; i++)
            {
                obj_geom->addTriangle(value[i][0], value[i][1], value[i][2]);
            }
            obj_geom->endModel();
            collisionObj = std::make_shared<fcl::CollisionObject<double>>(obj_geom);
        }
        else
        {
            std::cout << "Not vailed geom type" << std::endl;
            collisionObj.reset();
        }      

        tf_base = pLinkGeometry->tf_base;
        // cout<< "xxxxxxxxx"<<endl;
        // cout<< type << endl;
        // cout<< tf_base.matrix()<<endl;

    }

    bool setTF(Eigen::Isometry3d tf_link)
    {
        if (collisionObj.get() != NULL )
        {
            // Eigen::Isometry3d t = tf_base * tf_link ;
            // collisionObj->setTranslation(t.translation());
            // collisionObj->setRotation(t.rotation());

            collisionObj->setTransform( tf_link*tf_base );
            // collisionObj->setTransform( tf_link);
            return true;
        }
        else
        {
            return false;
        }
    }
};

typedef std::shared_ptr<CollisionLink> pCollisionLink_t;
typedef map<string, pCollisionLink_t> pCollisionLink_map_t;

class FCLObjGroup
{
    public:
    map<string, pCollisionLink_t> mLinkCollisionMap;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> mCollisionManager_ptr;
    vector<string> mCollisionLinks;

    public:
    FCLObjGroup() {}

    FCLObjGroup(map<string, pCollisionLink_t> LinkCollisionMap, map<string, Eigen::Isometry3d> tfMap)
    {
        registObjs(LinkCollisionMap);
        updatePose(tfMap);
        // getManager();
    }

    bool registObjs(map<string, pCollisionLink_t> LinkCollisionMap)
    {
        mLinkCollisionMap = LinkCollisionMap;

        mCollisionLinks.clear();
        for (auto it : mLinkCollisionMap)
        {
            mCollisionLinks.push_back(it.first);
        }
    }

    bool updatePose(map<string, Eigen::Isometry3d> tfMap)
    {
        // for (auto item : tfMap)
        // {
        //     string linkname;
        //     Eigen::Isometry3d tf;
        //     tie(linkname, tf) = item;
        //     auto clink_iter = mLinkCollisionMap.find(linkname);
        //     if (clink_iter != mLinkCollisionMap.end())
        //     {
        //         clink_iter->second->setTF(tf);
        //     }
        // }

        for (auto item : mLinkCollisionMap)
        {
            string linkname;
            pCollisionLink_t pCollisionLink;
            tie(linkname, pCollisionLink) = item;
            auto iter = tfMap.find(linkname);
            if (iter != tfMap.end())
            {
                pCollisionLink->setTF(iter->second);
            }
        }
        return true;
    }

    const pCollisionLink_t getCollisionLink(string linkName)
    {
         auto pcl = mLinkCollisionMap.find(linkName);
         if (pcl == mLinkCollisionMap.end())
         {
             return nullptr;
         }
         return pcl->second;
    }

    // std::shared_ptr<fcl::BroadPhaseCollisionManagerd> getManager()
    // {
    //     mCollisionManager_ptr = std::dynamic_pointer_cast<fcl::BroadPhaseCollisionManagerd>(std::make_shared<fcl::DynamicAABBTreeCollisionManager>());
    //     for (auto item : mLinkCollisionMap)
    //     {
    //         mCollisionManager_ptr->registerObject(item.second->obj.get());
    //     }
    //     return mCollisionManager_ptr;
    // }
};

class Robot_FCLObjGroup : public FCLObjGroup
{
    private:
    collisionPair_set_t mDisable_collisionss_set;
    collisionPair_set_t mCollision_pairs_set;

    public:
    void updateDisableCollisions(collisionPair_set_t disable_collisionss_set)
    {
        mDisable_collisionss_set = disable_collisionss_set;

        collisionPair_set_t allPair;
        int len = mCollisionLinks.size();
        for (int i = 0; i < len; i++)
        {
            for (int j = i + 1; j < len; j++)
            {
                collisionPair_t cp;
                cp.insert(mCollisionLinks[i]);
                cp.insert(mCollisionLinks[j]);
                allPair.insert(cp);
            }
        }

        set_difference(allPair.begin(),
                       allPair.end(),
                       mDisable_collisionss_set.begin(),
                       mDisable_collisionss_set.end(),
                       inserter(mCollision_pairs_set, mCollision_pairs_set.begin()));
    }

    bool checkCollision(string link1, string link2)
    {
        auto plg1 = getCollisionLink(link1);
        auto plg2 = getCollisionLink(link2);

        if(plg1 == nullptr || plg2 == nullptr )
        {
            return false;
        }

        fcl::CollisionRequest<double> request;

        fcl::CollisionResult<double> result;

        result.clear();

        fcl::CollisionObjectd *o1, *o2;
        o1 = plg1->collisionObj.get();
        o2 = plg2->collisionObj.get();

        fcl::collide<double>(o1, o2, request, result);

        bool rt = result.isCollision();

        #ifdef DEBUG
        cout<<"=========================="<<endl;
        cout<< o1->getTransform().matrix() <<endl;
        cout<< o2->getTransform().matrix() <<endl;
        if(rt)
            cout<< link1 <<" and "<< link2 <<" is in collision" <<endl;
        else
            cout<< link1 <<" and "<< link2 <<" is free" <<endl;
        #endif

        return rt;
    }

    bool checkSelfCollision()
    {
        for(auto p : mCollision_pairs_set)
        {
            // assert()
            // cout<< p.size() <<endl; 
            if(p.size() !=2 )
            {
                return false;
            }
            string link1, link2;
            
            auto it = p.begin();
            link1 = *it;
            it++;
            link2 = *it;
            
            if( checkCollision( link1, link2 ))
            {
                return true;
            }
        }
        return false;

    }
};

class CollisionDetector
{
    private:
    pCollisionLink_map_t mCollisionLinkMap;
    pCollisionLink_map_t mEnvLinkMap;
    pCollisionLink_map_t mRobotLinkMap;

    collisionPair_set_t mDisable_collisionss_set;

    set< pair<string, string> > mRe_cpSet; // 机器人与环境之间碰撞检测对

    public:
    FCLObjGroup envGroup;
    Robot_FCLObjGroup robotGroup;
    FCLObjGroup connectGroup;

    public:
    CollisionDetector(){}
    
    bool setup(pLinkGeometry_map_t pLinkGeometryMap, set<string>  envLinksSet, set<string>  robotLinksSet , collisionPair_set_t disable_collisionss_set)
    {
        
        registCollisionObjs(pLinkGeometryMap);
        mDisable_collisionss_set = disable_collisionss_set;
   
        registRobot(robotLinksSet);
        
        registEnv(envLinksSet);

        mRe_cpSet.clear();
        collisionPair_set_t cp_set;  // 机械臂与环境间需要做碰撞检测的link组合 
        for(string robot_link : robotGroup.mCollisionLinks )
            for( string env_link : envGroup.mCollisionLinks )
            { 
                collisionPair_t cp{ robot_link, env_link };
                if(  mDisable_collisionss_set.find(cp) == mDisable_collisionss_set.end()  )
                {
                    mRe_cpSet.insert( pair<string, string>(robot_link, env_link) );
                }
            }    

    }
    
    // 注册碰撞体
    bool registCollisionObjs(pLinkGeometry_map_t pLinkGeometryMap)
    {
        mCollisionLinkMap.clear();
        for(auto item : pLinkGeometryMap)
        {
            string name;
            pLinkGeometry_t pLinkGeometry;
            tie(name, pLinkGeometry) = item;
            pCollisionLink_t pCollisionLink = std::make_shared<CollisionLink>();
            pCollisionLink->loadGeometry(pLinkGeometry);
            

            // mCollisionLinkMap.insert(  pair<string, pCollisionLink_t>(name, pCollisionLink)  );
            mCollisionLinkMap[name] = pCollisionLink;
        }
    }
    // 注册机械臂部分
    bool registRobot(std::set<string> robotLinksSet)
    {
        mRobotLinkMap.clear();
        for(auto linkname: robotLinksSet)
        {
            if( mCollisionLinkMap.find(linkname ) !=  mCollisionLinkMap.end() )
            {
                mRobotLinkMap[linkname] = mCollisionLinkMap[linkname];
            }
        }
        robotGroup.registObjs(mRobotLinkMap);
        robotGroup.updateDisableCollisions(mDisable_collisionss_set);
        return true;
    }
    // 注册环境部分
    bool registEnv(std::set<string> envLinksSet)
    {
        mEnvLinkMap.clear();
        for(auto linkname: envLinksSet)
        {
            if( mCollisionLinkMap.find(linkname ) !=  mCollisionLinkMap.end() )
            {
                 mEnvLinkMap[linkname] = mCollisionLinkMap[linkname];
            }   
        }
        envGroup.registObjs(mEnvLinkMap);
        return true;
    }

    bool updateRobotPose(map<string, Eigen::Isometry3d> tfMap)
    {
        robotGroup.updatePose(tfMap);
    }

    bool updateEnvPose(map<string, Eigen::Isometry3d> tfMap)
    {
        envGroup.updatePose(tfMap);
    }

    bool checkCollision()
    {
        // 机器人自碰撞
        bool isSelfCollision = robotGroup.checkSelfCollision();
        if(isSelfCollision)return true;

        for( auto p : mRe_cpSet )
        {
            string link1, link2;
            tie(link1, link2) = p; 
            auto plg1 = robotGroup.getCollisionLink(link1);
            auto plg2 = envGroup.getCollisionLink(link2);


            fcl::CollisionRequest<double> request;

            fcl::CollisionResult<double> result;

            result.clear();

            fcl::CollisionObjectd *o1, *o2;
            o1 = plg1->collisionObj.get();
            o2 = plg2->collisionObj.get();

            fcl::collide<double>(o1, o2, request, result);

            bool rt = result.isCollision();

            #ifdef DEBUG
            cout<<"=========================="<<endl;
            cout<< o1->getTransform().matrix() <<endl;
            cout<< o2->getTransform().matrix() <<endl;
            if(rt)
                cout<< link1 <<" and "<< link2 <<" is in collision" <<endl;
            else
                cout<< link1 <<" and "<< link2 <<" is free" <<endl;
            #endif
            
            if(rt)
                return true;
        }

        return false;        
    }

    bool checkCollision(map<string, Eigen::Isometry3d> tfMap)
    {
        // for(auto a: tfMap)
        // {
        //     cout<<a.first<<endl<<a.second.matrix()<<endl;
        //     cout<<"asassas"<<endl;
        // }
        robotGroup.updatePose(tfMap);
        envGroup.updatePose(tfMap);
        return checkCollision();
    }

};