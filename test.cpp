/* 
 * 文件: Unit1.cpp 
 * 日期: 2008-12-31 8:59:53 
 * 作者: blackboy 
 * 描述: BGL学习 - 使用Bundled Properties 
*/     
   //--------------------------------------------------------------------------- 
#include <iostream> 
#include <string> 
#include <vector> 
#include <utility> 
#include <algorithm> 
#include <stdlib> 
#pragma hdrstop     
   #include <boost/graph/graph_traits.hpp> 
#include <boost/graph/adjacency_list.hpp> 
using namespace boost; 
//---------------------------------------------------------------------------     
   // 顶点是城市结构 
struct City 
{ 
    std::string name; 
    int population; 
    std::vector<int> zipcodes; 
};     
   // 边是高速公路结构 
struct Highway 
{ 
    std::string name; 
    double miles; 
    int speed_limit; 
    int lanes; 
    bool divided; 
};     
   // 声明自定义的图 
typedef boost::adjacency_list<boost::listS, boost::vecS, 
    boost::bidirectionalS, City, Highway> Map;     
   #pragma argsused 
int main(int argc, char* argv[]) 
{ 
    Map map;     
       // 向图中添加3顶点 
    City ct; 
    ct.name = "Beijing"; 
    ct.population = 12345; 
    ct.zipcodes.push_back(102206); 
    add_vertex(ct, map);     
       City ct2; 
    ct2.name = "ShangHai"; 
    ct2.population = 54321; 
    ct2.zipcodes.push_back(302546); 
    add_vertex(ct2, map);     
       City ct3; 
    ct3.name = "ShangHai"; 
    ct3.population = 54321; 
    ct3.zipcodes.push_back(302546); 
    add_vertex(ct3, map);     

    Highway hway; 
    hway.name = "BHWay"; 
    hway.miles = 1500; 
    hway.speed_limit = 160; 
    hway.lanes = 6; 
    hway.divided = true;     


    // 索引属性映射, 可获得顶点的索引属性 
    typename property_map<Map, vertex_index_t>::type 
        index = get(vertex_bundle, map);     
    
    // 如果使用下面的属性映射, 则可获得特定的顶点属性值: 城市名 
    typename property_map<Map, std::string City::*>::type 
        Names = get(&City::name, map); 

    // 如果要获得所有顶点属性值的映射, 可用get(vertex_bundle, map)  
       // 顶点描述符 
    typedef typename graph_traits<Map>::vertex_descriptor Vertex; 
    // 顶点迭代器 
    typedef graph_traits<Map>::vertex_iterator vertex_iter; 
    // 遍历图中所有顶点 
    std::pair<vertex_iter, vertex_iter> vp; 
    for(vp=vertices(map); vp.first!=vp.second; ++vp.first) 
    { 
        // 输出顶点的索引 
        std::cout << index[*vp.first] << " - "; 
        Vertex v = *vp.first; 
        // 使用[]操作符获得自定义的顶点类型, 并输出bundled属性值 
        std::cout << map[v].name << ", " << map[v].population << ", " 
            << map[v].zipcodes[0] << std::endl; 
    }     
       City ct4; 
    ct4.name = "NewYork"; 
    ct4.population = 65321; 
    ct4.zipcodes.push_back(258415);     
       // 向图中添加一条边 
    vp = vertices(map); 
    Vertex v1 = *vp.first; 
    vp.first++; 
    Vertex v2 = *vp.first; 
    add_edge(v1, v2, hway, map);     
       // 边迭代器 
    typedef graph_traits<Map>::edge_iterator edge_iter; 
    std::pair<edge_iter, edge_iter> ep; 
    ep = edges(map); 
    // 边描述符 
    typedef graph_traits<Map>::edge_descriptor Edge; 
    Edge e = *ep.first; 
    // 输出第一条边的bundled各属性值 
    std::cout << map[e].name << ", " << map[e].miles << ", " 
        << map[e].speed_limit << ", " << map[e].divided << std::endl;     
       system("PAUSE"); 
    return 0; 
} 
//---------------------------------------------------------------------------   


typedef boost::adjacency_list<
    boost::listS, boost::vecS, boost::bidirectionalS,
    // Vertex properties
    boost::property< boost::vertex_name_t, std::string, boost::property<population_t, int, boost::property<zipcodes_t, std::vector<int> > > >,
    // Edge properties
    boost::property<boost::edge_name_t, std::string,
    boost::property<boost::edge_weight_t, double,
    boost::property<edge_speed_limit_t, int,
    boost::property<edge_lanes_t, int,
    boost::property<edge_divided, bool> > > > > >
  Map;