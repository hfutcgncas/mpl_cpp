#ifndef __TF_GRAPH_HPP__
#define __TF_GRAPH_HPP__

#include <string>
#include <vector>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/topological_sort.hpp>

// #include <boost/graph/labeled_graph.hpp>   // 可以考虑采用

// #include"robotModel_utils.hpp"

#include "Eigen/Geometry"

namespace tf_Graph
{
using namespace boost;
using namespace std;

// struct vertex_link_t
// {
//     typedef vertex_property_tag kind;
// };
// struct edge_Joint_t
// {
//     typedef edge_property_tag kind;
// };

// typedef property<vertex_link_t, Link> VertexLinkProperty;
// typedef property<edge_Joint_t, Joint> EdgeJointProperty;

class Frame
{
public:
    Eigen::Isometry3d rt2base;
    string name;

    Frame()
    {
        // name = ""
        rt2base = Eigen::Isometry3d::Identity();
    }

    Frame(string Name) : name(Name), rt2base(Eigen::Isometry3d::Identity())
    {
    }

    virtual ~Frame() {}

    Frame(const Frame &src) : rt2base(src.rt2base), name(src.name)
    {
    }

    Frame &operator=(const Frame &src)
    {
        name = src.name;
        rt2base = src.rt2base;
    }
};

class TF
{
public:
    Eigen::Isometry3d trans;
    string name;
    string parent;
    string child;

    TF()
    {
        trans = Eigen::Isometry3d::Identity();
    }
    TF(const TF &src) : trans(src.trans), name(src.name)
    {
    }

    TF &operator=(const TF &src)
    {
        name = src.name;
        trans = src.trans;
        return *this;
    }

    virtual ~TF() {}
};

typedef std::shared_ptr<Frame> pFrame_t;
typedef std::shared_ptr<TF> pTF_t;

// typedef adjacency_list<listS, vecS, directedS, pFrame_t, pTF_t> DiGraph;

typedef adjacency_list<listS, vecS, bidirectionalS, pFrame_t, pTF_t> DiGraph;

// typedef labeled_graph<DiGraph, std::string> LGraph;

typedef typename graph_traits<DiGraph>::vertex_descriptor vertex_descriptor_t;
typedef typename graph_traits<DiGraph>::edge_descriptor edge_descriptor_t;

typedef typename graph_traits<DiGraph>::edge_iterator edge_iterator_t;

typedef std::list<vertex_descriptor_t> TFOrder_t;
typedef typename graph_traits<DiGraph>::out_edge_iterator out_edge_iterator_t;
typedef typename graph_traits<DiGraph>::in_edge_iterator in_edge_iterator_t;

class TF_Graph
{

    public:

    DiGraph g;
    // LGraph g;
    map<string, vertex_descriptor_t> Vmap;
    map<string, edge_descriptor_t> Emap;

    map<string, Eigen::Isometry3d> tfMap;

    TFOrder_t TFOrder;

    TFOrder_t *updateTFOrder()
    {
        TFOrder.clear();
        topological_sort(g, std::front_inserter(TFOrder));
        return &TFOrder;
    }

    bool updateFrame_trans()
    {
        if (TFOrder.size() == 0)
        {
            return false;
        }

        out_edge_iterator_t out_i, out_end;

        for (vertex_descriptor_t v : TFOrder)
        {
            boost::tie(out_i, out_end) = out_edges(v, g);

            for (; out_i != out_end; out_i++)
            {
                edge_descriptor_t e = *out_i;
                vertex_descriptor_t child_v = target(e, g);
                g[child_v]->rt2base = g[v]->rt2base * g[e]->trans;

                // std::cout<<g[*out_i].name<<endl;
                // std::cout<<"e"<<endl<<g[e].name<<endl<<g[e].trans.matrix()<<endl;
                // std::cout<<g[child_v].name<<endl;
                // std::cout<<"cv"<<g[child_v].rt2base.matrix()<<endl;
            }
            // std::cout<<"================"<<endl;
        }

        updateTfMap();

        return true;
    }

    // 查 ===============================================
    bool isFrameInTree(const string frameName)
    {
        DiGraph::vertex_iterator vertexIt, vertexEnd;
        tie(vertexIt, vertexEnd) = vertices(g);
        for (; vertexIt != vertexEnd; ++vertexIt)
        {
            if (frameName == g[*vertexIt]->name)
            {
                return true;
            }
        }
        return false;
    }

    pFrame_t getFrame_p(const string frameName)
    {
        return g[Vmap.at(frameName)];
    }

    pTF_t getTF_p(const string tfName)
    {
        return g[Emap.at(tfName)];
    }

    pFrame_t getFrame_p_safe(const string frameName)
    {
        DiGraph::vertex_iterator vertexIt, vertexEnd;
        tie(vertexIt, vertexEnd) = vertices(g);
        for (; vertexIt != vertexEnd; ++vertexIt)
        {
            if (frameName == g[*vertexIt]->name)
            {
                return g[*vertexIt];
            }
        }
        return nullptr;
    }

    pTF_t getTF_p_safe(const string tfName)
    {
        DiGraph::edge_iterator edgeIt, edgeEnd;
        tie(edgeIt, edgeEnd) = boost::edges(g);
        for (; edgeIt != edgeEnd; ++edgeIt)
        {
            if (tfName == g[*edgeIt]->name)
            {
                return g[*edgeIt];
            }
        }
        return nullptr;
    }

    Eigen::Isometry3d getTrans(string target_frameName, string base_frameName)
    {
        Eigen::Isometry3d target = getFrame_p(target_frameName)->rt2base;
        Eigen::Isometry3d base = getFrame_p(base_frameName)->rt2base;
        return base.inverse() * target;
    }

    string getRootFrameName()
    {
        updateTFOrder();
        assert(~TFOrder.empty());

        // for (vertex_descriptor_t v : TFOrder)
        // {
        //     // return the first item
        //     return g[v]->name;
        // }
        vertex_descriptor_t v = *TFOrder.begin();
        return g[v]->name;
    }
    // 判断是否是叶子节点
    bool isLeafFrame(const string frameName)
    {
        vertex_descriptor_t v = Vmap[frameName];
        size_t a = out_degree(v, g);
        return (a == 0);
    }

    bool updateVEmap()
    {
        DiGraph::vertex_iterator vertexIt, vertexEnd;
        out_edge_iterator_t out_i, out_end;

        Vmap.erase(Vmap.begin(), Vmap.end());
        Emap.erase(Emap.begin(), Emap.end());

        tie(vertexIt, vertexEnd) = vertices(g);
        for (; vertexIt != vertexEnd; ++vertexIt)
        {
            Vmap.insert(std::pair<string, vertex_descriptor_t>(g[*vertexIt]->name, *vertexIt));
            boost::tie(out_i, out_end) = out_edges(*vertexIt, g);
            for (; out_i != out_end; ++out_i)
            {
                Emap.insert(std::pair<string, edge_descriptor_t>(g[*out_i]->name, *out_i));
            }
        }
    }

    bool updateTfMap()
    {
        for(auto item:Vmap)
        {
            tfMap[item.first] = g[item.second]->rt2base;
        }
    }
    
    
    

    // 删 ======================================================
    // 删除frame。只删除叶子节点，对非叶子节点无操作
    bool rmFrame(const string frameName)
    {
        vertex_descriptor_t v = Vmap[frameName];
        in_edge_iterator_t in_i, in_end;
        if (isLeafFrame(frameName))
        {
            clear_vertex(v, g);
            remove_vertex(v, g);

            updateVEmap(); //每次删除就要完全重建一次Vmap和Emap以实现同步。有没有更好的方法？
            return true;
        }
        else
        {
            return false;
        }
    }

    // 删除frame及其后续所有frame
    bool rmFrame_recursive(const string frameName)
    {
        if (rmFrame(frameName))
        {
            return true;
        }
        else
        {
            // BGL 对一个节点的所有的子节点做clear_edges后，最后一个节点的边无法删除，不知为何
            // 所以这里先在父节点把出边都删掉，再清理各个字节点的边。
            vector<string> child_v_name_list;
            vector<string> child_j_name_list;
            vertex_descriptor_t child_v;
            out_edge_iterator_t out_i, out_end;
            boost::tie(out_i, out_end) = out_edges(Vmap[frameName], g);
            for (; out_i != out_end; ++out_i)
            {
                child_j_name_list.push_back(g[*out_i]->name);
                child_v = target(*out_i, g);
                child_v_name_list.push_back(g[child_v]->name);
            }

            clear_out_edges(Vmap[frameName], g);

            for (auto v_name : child_v_name_list)
            {
                rmFrame_recursive(v_name);
            }
            bool success = rmFrame(frameName);
            assert(success);
            return true;
        }
    }

    // 增 ======================================================
    // 要求parentName必须存在
    bool add_Frame(pFrame_t pNewFrame, string parentName, pTF_t tf)
    {
        // 要求newFrameName 不在现有树上 ,  且parentName在现有树上
        if ((Vmap.find(pNewFrame->name) != Vmap.end()) || (Vmap.find(parentName) == Vmap.end()))
        {
            return false;
        }

        pFrame_t parentFrame = getFrame_p(parentName);

        vertex_descriptor_t v = boost::add_vertex(pNewFrame, g);
        Vmap.insert(pair<string, vertex_descriptor_t>(pNewFrame->name, v));

        edge_descriptor_t e;
        bool success;
        boost::tie(e, success) = boost::add_edge(Vmap.at(parentName), v, tf, g);
        if (success)
        {
            Emap.insert(pair<string, edge_descriptor_t>(tf->name, e));
            return true;
        }
        return false;
    }

    bool add_Frame(string newFrameName, string parentName, Eigen::Isometry3d trans)
    {
        // 要求newFrameName 不在现有树上
        if (Vmap.find(newFrameName) != Vmap.end())
        {
            return false;
        }

        vertex_descriptor_t v;
        if (Vmap.find(parentName) != Vmap.end())
        {
            v = Vmap.at(parentName);
            pFrame_t pParent = g[v];
        }
        else
        {
            pFrame_t pParent = std::make_shared<Frame>(parentName);
            v = boost::add_vertex(pParent, g);
            Vmap.insert(pair<string, vertex_descriptor_t>(parentName, v));
        }

        pFrame_t pNewFrame = std::make_shared<Frame>(newFrameName);
        vertex_descriptor_t v_new = boost::add_vertex(pNewFrame, g);
        Vmap.insert(pair<string, vertex_descriptor_t>(newFrameName, v_new));

        string jointName = parentName + "-" + newFrameName + "-joint1";
        pTF_t ptf = std::make_shared<tf_Graph::TF>();
        ptf->name = jointName;
        ptf->parent = parentName;
        ptf->child = newFrameName;
        ptf->trans = trans;

        edge_descriptor_t e;
        bool success;
        boost::tie(e, success) = boost::add_edge(v, v_new, ptf, g);
        if (success)
        {
            Emap.insert(pair<string, edge_descriptor_t>(ptf->name, e));
            return true;
        }
        return false;
    }

    pair<pFrame_t, pTF_t> getParentVE(string frameName)
    {
        vertex_descriptor_t vf, vp;
        in_edge_iterator_t in_i, in_end;

        vf = Vmap.at(frameName);
        boost::tie(in_i, in_end) = in_edges(Vmap[frameName], g);
        if (in_i == in_end)
        {
            return pair<pFrame_t, pTF_t>(g[vf], nullptr);
        }
        else
        {
            vp = source(*in_i, g);
            return pair<pFrame_t, pTF_t>(g[vp], g[*in_i]);
        }
    }

    set<pair<pFrame_t, pTF_t>> getChildVE(string frameName)
    {
        vertex_descriptor_t vf, vc;
        out_edge_iterator_t out_i, out_end;
        set<pair<pFrame_t, pTF_t>> rt;

        vf = Vmap.at(frameName);
        boost::tie(out_i, out_end) = out_edges(Vmap[frameName], g);
        if (out_i == out_end)
        {
            return rt;
        }
        else
        {
            for (; out_i != out_end; out_i++)
            {
                vc = target(*out_i, g);
                rt.insert(pair<pFrame_t, pTF_t>(g[vc], g[*out_i]));
            }

            return rt;
        }
    }

    bool ChangeParent(string frameName, string newParentName)
    {
        updateVEmap();
        updateFrame_trans();

        pFrame_t pf, pOldParenf;
        pTF_t ptf;
        pf = getFrame_p(frameName);
        tie(pOldParenf, ptf) = getParentVE(frameName);

        Eigen::Isometry3d newtrans = getTrans(frameName, newParentName);
        remove_edge(Emap.at(ptf->name), g);
        // modify old tf to new tf
        ptf->name = newParentName + "-" + frameName + "-joint";
        ptf->parent = newParentName;
        ptf->child = frameName;
        ptf->trans = newtrans;
        add_edge(Vmap.at(newParentName), Vmap.at(frameName), ptf, g);

        updateVEmap();
        updateTFOrder();
        updateFrame_trans();
        return true;
    }

    string getRoot(string frameName)
    {
        pFrame_t pOldParenf;
        pTF_t ptf;
        string name_tmp = frameName;
        do
        {
            tie(pOldParenf, ptf) = getParentVE(name_tmp);
            name_tmp = pOldParenf->name;
        } while (ptf != nullptr);

        return name_tmp;
    }

    // todo
    // isTree
    // getparent
    // getparentedge
    //
};

} // namespace tf_Graph

#endif