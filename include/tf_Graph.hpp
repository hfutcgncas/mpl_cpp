#ifndef __TF_GRAPH_HPP__
#define __TF_GRAPH_HPP__


#include <string>
#include <vector>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>

#include"robotModel_utils.hpp"



namespace RobotModel
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
typedef adjacency_list<listS, vecS, directedS, Link, Joint> DiGraph;
typedef typename graph_traits<DiGraph>::vertex_descriptor vertex_descriptor_t;
typedef typename graph_traits<DiGraph>::edge_descriptor edge_descriptor_t;


class TF_Graph
{


public:
    DiGraph g;
    map<string, vertex_descriptor_t> Vmap;
    map<string, edge_descriptor_t> Emap;

    void build(const map<string, Joint> JointMap, const map<string, Link> LinkMap)
    {
        for (auto item : LinkMap)
        {
            vertex_descriptor_t v = boost::add_vertex(item.second, g);
            Vmap.insert(pair<string, vertex_descriptor_t>(item.first, v));
        }

        for (auto item : JointMap)
        {
            auto v1 = Vmap[item.second.parent];
            auto v2 = Vmap[item.second.child];
            // Add edges
            std::pair<edge_descriptor_t, bool> e = boost::add_edge(v1, v2, g);
            g[e.first] = item.second;
            Emap.insert(pair<string, edge_descriptor_t>(item.first, e.first));
        }
    }

    Link *getLink_p(const string linkname)
    {
        vertex_descriptor_t v = Vmap[linkname];
        return &g[v];
    }

    Joint *getJoint_p(const string jointName)
    {
        edge_descriptor_t e = Emap[jointName];
        return &g[e];
    }

    // todo
    bool add_TF(Joint j, Link l)
    {
        return false;
    }
    bool rm_Link(string linkName)
    {
        return false;
    }
    bool modify_TF(Joint j, Link l)
    {
        return false;
    }
    bool update()
    {
        return false;
    }

};

} // namespace RobotModel

#endif