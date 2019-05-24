#include <iostream>

#include <vector>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>

// Create a struct to hold properties for each vertex
typedef struct vertex_properties
{
    std::string label;
    int p1;
} vertex_properties_t;

// Create a struct to hold properties for each edge
typedef struct edge_properties
{
    std::string label;
    //int   p1;
      double   weight;
} edge_properties_t;

// Define the type of the graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, vertex_properties_t, edge_properties_t> graph_t;
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor_t;
typedef graph_t::edge_descriptor   edge_descriptor_t;
typedef boost::property_map<graph_t, boost::vertex_index_t>::type index_map_t;
typedef boost::iterator_property_map<vertex_descriptor_t *, index_map_t *, vertex_descriptor_t, vertex_descriptor_t &> predecessor_map_t;

//  The graph, with edge weights labeled.
//
//   v1  --(1)--  v2
//   |  \_        |
//   |    \       |
//  (1)    (3)   (2)
//   |        \_  |
//   |          \ |
//   v4  --(1)--  v3
//
//

int main(int, char *[])
{ 
    // Create a graph object
    graph_t g;

    // Add vertices
    vertex_descriptor_t v1 = boost::add_vertex(g);
    vertex_descriptor_t v2 = boost::add_vertex(g);

    //vertex_descriptor_t v11 = boost::add_vertex(g);
      //vertex_descriptor_t v22 = boost::add_vertex(g);

    vertex_descriptor_t v3 = boost::add_vertex(g);
    vertex_descriptor_t v4 = boost::add_vertex(g);

    vertex_descriptor_t v5 = boost::add_vertex(g);

    // Set vertex properties
      g[v1].p1 = 1;
    g[v1].label = "v1";
    g[v2].p1 = 2;
    g[v2].label = "v2";

    //g[v11].p1 = 2;  g[v11].label = "v11";
      //g[v22].p1 = 1;  g[v22].label = "v22";

    g[v3].p1 = 3;
    g[v3].label = "v3";
    g[v4].p1 = 4;
    g[v4].label = "v4";

    g[v5].p1 = 5;
    g[v5].label = "v5";

    // Add edges
      std::pair<edge_descriptor_t, bool> e012 = boost::add_edge(v1, v2, g);
    std::pair<edge_descriptor_t, bool> e021 = boost::add_edge(v2, v1, g);

    std::pair<edge_descriptor_t, bool> e023 = boost::add_edge(v2, v3, g);
    std::pair<edge_descriptor_t, bool> e034 = boost::add_edge(v3, v4, g);
    std::pair<edge_descriptor_t, bool> e041 = boost::add_edge(v4, v1, g);
    std::pair<edge_descriptor_t, bool> e013 = boost::add_edge(v1, v3, g);

    std::pair<edge_descriptor_t, bool> e025 = boost::add_edge(v2, v5, g);

    // Set edge properties
    //  g[e01.first].p1 = 1;
    //  g[e02.first].p1 = 2;
    //  g[e03.first].p1 = 3;
    //  g[e04.first].p1 = 4;
    //  g[e05.first].p1 = 5;

      g[e012.first]
        .weight = 1;
    g[e021.first].weight = 1;

    g[e023.first].weight = 2;
    g[e034.first].weight = 3;
    g[e041.first].weight = 5;
    g[e013.first].weight = 6;

    g[e025.first].weight = 7;

    g[e012.first].label = "v1-v2";
    g[e021.first].label = "v2-v1";
    g[e023.first].label = "v2-v3";
    g[e034.first].label = "v3-v4";
    g[e041.first].label = "v4-v1";
    g[e013.first].label = "v1-v3";

    // Print out some useful information
      std::cout << "Graph:" << std::endl;
    boost::print_graph(g, boost::get(&vertex_properties_t::p1, g));
    std::cout << "num_verts: " << boost::num_vertices(g) << std::endl;
    std::cout << "num_edges: " << boost::num_edges(g) << std::endl;

    // BGL Dijkstra's Shortest Paths here...
    std::vector<double> distances(boost::num_vertices(g));
    std::vector<vertex_descriptor_t> predecessors(boost::num_vertices(g));

    //Vertex s = *(vertices(G).first);
      //       // invoke variant 2 of Dijkstra's algorithm
  //       dijkstra_shortest_paths(G, s, distance_map(&d[1]));

  //       std::cout << "distances from start vertex:" << std::endl;
  //       graph_traits<Graph>::vertex_iterator vi;
  //       for(vi = vertices(G).first; vi != vertices(G).second; ++vi)
  //           std::cout << "distance(" << index[*vi] << ") = "
  //           << d[*vi] << std::endl;
  //       std::cout << std::endl;
  vertex_descriptor_t vSource;
    vertex_descriptor_t vtarget;
    boost::graph_traits<graph_t>::vertex_iterator vi;
    bool bFindSource = false;
    bool bFindTarget = false;
    for (vi = boost::vertices(g).first; vi != boost::vertices(g).second; ++vi)
         
        {
                   if (g[*vi].p1 == 4)
      
            {
                           vSource = *vi;
                           bFindSource = true;
                      
            }
                   if (g[*vi].p1 == 5)
      
            {
                           vtarget = *vi;
                           bFindTarget = true;
                      
            }

                   if (bFindSource && bFindTarget)
      
            {
                           break;
                      
            }
                   else       
            {
                           return 0;
                      
            }
             
        }

    boost::dijkstra_shortest_paths(g, vSource,
                                 boost::weight_map(boost::get(&edge_properties_t::weight, g))
                                 .distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g)))
                                 .predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, g)))
                                 );

    // Extract the shortest path from v1 to v3.
      typedef std::vector<edge_descriptor_t> path_t;
    path_t path;

    vertex_descriptor_t v = vtarget;
    for (vertex_descriptor_t u = predecessors[v]; u != v; v = u, u = predecessors[v])
    {
            std::pair<edge_descriptor_t, bool> edge_pair = boost::edge(u, v, g);
            path.push_back(edge_pair.first);
         
    }

    std::cout << std::endl;
    //std::cout << "Shortest Path from v1 to v3:" << std::endl;
      //std::cout << "distance v1 to v3:" <<distances[3]<<std::endl;
  double distance = 0;
    for (path_t::reverse_iterator riter = path.rbegin(); riter != path.rend(); ++riter)
    {
        vertex_descriptor_t u_tmp = boost::source(*riter, g);
        vertex_descriptor_t v_tmp = boost::target(*riter, g);
        edge_descriptor_t   e_tmp = boost::edge(u_tmp, v_tmp, g).first;
        distance += g[e_tmp].weight;
        std::cout << "  " << g[u_tmp].p1 << " -> " << g[v_tmp].p1 << "    (weight: " << g[e_tmp].weight << ")" << std::endl;
    }
    std::cout << "distance v1 to v3:" << distance << std::endl;
    return 0;
}
