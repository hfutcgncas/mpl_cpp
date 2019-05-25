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

    Frame() {}
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

    TF() {}
    TF(const TF &src) : trans(src.trans), name(src.name)
    {
    }

    TF &operator=(const TF &src)
    {
        name = src.name;
        trans = src.trans;
    }
};

typedef adjacency_list<listS, vecS, directedS, Frame, TF> DiGraph;
typedef typename graph_traits<DiGraph>::vertex_descriptor vertex_descriptor_t;
typedef typename graph_traits<DiGraph>::edge_descriptor edge_descriptor_t;

typedef std::list<vertex_descriptor_t> TFOrder_t;

class TF_Graph
{

public:
    DiGraph g;
    map<string, vertex_descriptor_t> Vmap;
    map<string, edge_descriptor_t> Emap;
    TFOrder_t TFOrder;
    
    TFOrder_t* updateTFOrder()
    {
        TFOrder.clear();
        topological_sort(g, std::front_inserter(TFOrder));
        return &TFOrder;
    }

    bool updateFtame_trans()
    {
        if(TFOrder.size() == 0)
        {
            return false;
        }
        
        for( auto v : TFOrder )
        {
            std::cout<<g[v].name <<endl;
        }

        return true;
    }

    Frame *getFrame_p(const string frameName)
    {
        vertex_descriptor_t v = Vmap[frameName];
        return &g[v];
    }

    TF *getTF_p(const string tfName)
    {
        edge_descriptor_t e = Emap[tfName];
        return &g[e];
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