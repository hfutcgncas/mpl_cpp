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
        rt2base = Eigen::Isometry3d::Identity();
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

typedef typename graph_traits<DiGraph>::vertex_descriptor vertex_descriptor_t;
typedef typename graph_traits<DiGraph>::edge_descriptor edge_descriptor_t;

typedef std::list<vertex_descriptor_t> TFOrder_t;
typedef typename graph_traits<DiGraph>::out_edge_iterator out_edge_iterator_t;
typedef typename graph_traits<DiGraph>::in_edge_iterator in_edge_iterator_t;

class TF_Graph
{

public:
    DiGraph g;
    map<string, vertex_descriptor_t> Vmap;
    map<string, edge_descriptor_t> Emap;


    TFOrder_t TFOrder;

    TFOrder_t *updateTFOrder()
    {
        TFOrder.clear();
        topological_sort(g, std::front_inserter(TFOrder));
        return &TFOrder;
    }

    bool updateFtame_trans()
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

        return true;
    }

    pFrame_t getFrame_p(const string frameName)
    {
        return g[Vmap[frameName]];
    }

    pTF_t getTF_p(const string tfName)
    {
        return g[Emap[tfName]];
    }

    // 判断是否是叶子节点
    bool isLeafFrame(const string frameName)
    {
        vertex_descriptor_t v = Vmap[frameName];
        size_t a = out_degree(v, g);
        return (a==0);
    }

    // 删除frame。只删除叶子节点，对非叶子节点无操作
    bool rmFrame(const string frameName)
    {
        vertex_descriptor_t v = Vmap[frameName];
        if(isLeafFrame(frameName))
        {
            clear_vertex(v, g);
            remove_vertex(v, g);
            Vmap.erase(frameName);
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
        if(rmFrame(frameName))
        {
            return true;
        }
        else
        {
            vertex_descriptor_t v = Vmap[frameName];
            out_edge_iterator_t out_i, out_end;
            boost::tie(out_i, out_end) = out_edges(v, g); 
            for (; out_i != out_end; out_i++)
            {
                edge_descriptor_t e = *out_i;
                vertex_descriptor_t child_v = target(e, g);
                rmFrame_recursive( g[child_v]->name );
            }
            return true;
        }
    }





    // // todo
    // bool add_TF(Joint j, Link l)
    // {
    //     return false;
    // }
    // bool rm_Link(string linkName)
    // {
    //     return false;
    // }
    // bool modify_TF(Joint j, Link l)
    // {
    //     return false;
    // }
    // bool update()
    // {
    //     return false;
    // }
};

} // namespace tf_Graph

#endif