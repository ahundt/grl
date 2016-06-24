/// Copyright (c) 2014-2016 Andrew Hundt, Johns Hopkins University
/// 
/// @author Andrew Hundt <ATHundt@gmail.com>
/// 
/// Dual licensed under the BSD or Apache License, Version 2.0 
/// (the "Licenses") licenses at users option. Contributions shall be made such 
/// that users can continue to choose between the Licenses. 
/// You may not use this project except in compliance with the Licenses.
/// 
/// BSD license:
/// ------------
/// 
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met: 
/// 
/// 1. Redistributions of source code must retain the above copyright notice, this
///    list of conditions and the following disclaimer. 
/// 2. Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution. 
/// 
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
/// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
/// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/// 
/// The views and conclusions contained in the software and documentation are those
/// of the authors and should not be interpreted as representing official policies, 
/// either expressed or implied, of the grl Project.
/// 
/// Apache v2 license:
/// ------------------
/// 
/// You may obtain a copy of the Apache License, Version 2.0 (the "License") at
/// 
///     http://www.apache.org/licenses/LICENSE-2.0
/// 
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.

#ifndef RTREE_PLANNER_HPP_
#define RTREE_PLANNER_HPP_

#include <vector>

#include <boost/random/mersenne_twister.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <boost/pending/indirect_cmp.hpp>
#include <boost/range/irange.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/fill.hpp>


#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/geometries/adapted/boost_array.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/sign.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

BOOST_GEOMETRY_REGISTER_BOOST_ARRAY_CS(cs::cartesian)
    


namespace plan {
    
// Define properties for vertex
// source boost geometry graph example: https://github.com/boostorg/geometry/blob/bd9455207bb0012237c1853c3281ea95f146f91c/example/07_b_graph_route_example.cpp
template <typename Point>
struct bg_vertex_property
{
    bg_vertex_property()
    {
        boost::geometry::assign_zero(location);
    }
    bg_vertex_property(Point const& loc)
        : location(loc)
    {
    }

    Point location;
};

// Define properties for edge
// source boost geometry graph example: https://github.com/boostorg/geometry/blob/bd9455207bb0012237c1853c3281ea95f146f91c/example/07_b_graph_route_example.cpp
template <typename Line>
struct bg_edge_property
{
    bg_edge_property(Line const& line)
        : length(boost::geometry::length(line))
        , m_line(line)
    {
    }

    inline Line const& line() const
    {
        return m_line;
    }

    double length;
private :
    Line m_line;
};

    
    //typedef bg::model::point<double, 3, bg::cs::cartesian> point;
    typedef boost::array<double,6> ArmPos;
    typedef ArmPos point_type;
    typedef bg::model::box<point_type> box;
    //typedef bg::model::referring_segment<point_type> line_type; /// @todo maybe better to use refs?
    typedef bg::model::segment<point_type> line_type;
    typedef line_type edge_type;
    //typedef bg::model::polygon<point, false, false> polygon; // ccw, open polygon
    
    typedef boost::adjacency_list
            <
                boost::vecS, boost::vecS, boost::undirectedS
                , bg_vertex_property<point_type> // bundled
                , bg_edge_property<line_type>
            > graph_type;

    typedef boost::graph_traits<graph_type>::vertex_descriptor vertex_descriptor_type;
    typedef bg_vertex_property<point_type>                     vertex_property_type;
    typedef bg_edge_property<edge_type>                        edge_property_type;

    typedef std::pair<point_type, vertex_descriptor_type> rtree_value;
    typedef boost::geometry::index::rtree<rtree_value,bgi::rstar<16, 4> > knn_rtree_type;
    
} // namespace plan


BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<plan::ArmPos>)


template<typename T>
inline T normalizeRadiansPiToMinusPi(T rad)
{
  // copy the sign of the value in radians to the value of pi
  T signedPI = boost::math::copysign(boost::math::constants::pi<T>(),rad);
  // set the value of rad to the appropriate signed value between pi and -pi
  rad = std::fmod(rad+signedPI,(boost::math::constants::two_pi<T>())) - signedPI;

  return rad;
} 


// functor for getting sum of previous result and square of current element
// source: http://stackoverflow.com/questions/1326118/sum-of-square-of-each-elements-in-the-vector-using-for-each
template<typename T>
struct square
{
    T operator()(const T& Left, const T& Right) const
    {   
        return (Left + Right*Right);
    }
};

namespace boost { namespace geometry {
    

/// distance on an n-torus, so any dimensions offset by 2pi are equal and distances wrap
/// @example comparable_distance(pi-.01,-pi+.01) == .02 radians 
double comparable_distance(plan::ArmPos const& p1, plan::ArmPos const& p2 ) {
    plan::ArmPos diff;
    boost::transform(p1,p2,diff.begin(),std::minus<plan::ArmPos::value_type>());
    boost::transform(diff,diff.begin(),&normalizeRadiansPiToMinusPi<plan::ArmPos::value_type>);
    return boost::accumulate(diff,0,square<plan::ArmPos::value_type>());
}



/// distance between a a point and an axis aligned "box" on the surface of an n-torus
//  so any dimensions offset by 2pi are equal and distances wrap
template<typename Box>
double comparable_distance(plan::ArmPos const& armpos, Box const& box ){
    namespace bg = boost::geometry;
    plan::ArmPos normAP = normalizeRadiansPiToMinusPi(armpos);
    plan::ArmPos mindiff;
    boost::transform(normAP,bg::get<bg::min_corner>(box),mindiff.begin(),std::minus<plan::ArmPos::value_type>());
    boost::transform(mindiff,mindiff.begin(),&normalizeRadiansPiToMinusPi<plan::ArmPos::value_type>);
    plan::ArmPos maxdiff;
    boost::transform(normAP,bg::get<bg::max_corner>(box),maxdiff.begin(),std::minus<plan::ArmPos::value_type>());
    boost::transform(maxdiff,maxdiff.begin(),&normalizeRadiansPiToMinusPi<plan::ArmPos::value_type>);
    
    plan::ArmPos::value_type final_distance = 0.0;
    for(int i = 0; i < armpos.size(); ++i){
        if(mindiff[i] >= 0.0 && maxdiff[i] <= 0.0) continue; // between the min and max means "in the box" for this dimension
        plan::ArmPos::value_type min_dist = std::min(std::abs(mindiff[i]),std::abs(maxdiff[i]));
        final_distance+=min_dist*min_dist;
    }
    
    return final_distance;
//    diff (min<D> - p<D>), (p<D> - max<D>)
}

}} // namespace boost::geometry


/// 
/// source boost geometry graph example: https://github.com/boostorg/geometry/blob/bd9455207bb0012237c1853c3281ea95f146f91c/example/07_b_graph_route_example.cpp
template <typename Graph, typename Route>
inline void add_edge_to_route(Graph const& graph,
            typename boost::graph_traits<Graph>::vertex_descriptor vertex1,
            typename boost::graph_traits<Graph>::vertex_descriptor vertex2,
            Route& route)
{
    std::pair
        <
            typename boost::graph_traits<Graph>::edge_descriptor,
            bool
        > opt_edge = boost::edge(vertex1, vertex2, graph);
    if (opt_edge.second)
    {
        // Get properties of edge and of vertex
        plan::edge_property_type const& edge_prop = graph[opt_edge.first];
        plan::bg_vertex_property
            <
                typename boost::geometry::point_type<plan::edge_type>::type
            > const& vertex_prop = graph[vertex2];

        // Depending on how edge connects to vertex, copy it forward or backward
        if (boost::geometry::equals(*bg::segment_view<plan::edge_type>(edge_prop.line()).begin(), vertex_prop.location))
        {
            std::copy(bg::segment_view<plan::edge_type>(edge_prop.line()).begin(), bg::segment_view<plan::edge_type>(edge_prop.line()).end(),
                std::back_inserter(route));
        }
        else
        {
            std::reverse_copy(bg::segment_view<plan::edge_type>(edge_prop.line()).begin(), bg::segment_view<plan::edge_type>(edge_prop.line()).end(),
                std::back_inserter(route));
        }
    }
}

/// After running dijkstra's algorithm build list of shortest path steps between start and end position.
/// @note there will be duplicate positions that should be removed.
/// source boost geometry graph example: https://github.com/boostorg/geometry/blob/bd9455207bb0012237c1853c3281ea95f146f91c/example/07_b_graph_route_example.cpp
template <typename Graph, typename Route>
inline void build_route(Graph const& graph,
            std::vector<typename boost::graph_traits<Graph>::vertex_descriptor> const& predecessors,
            typename boost::graph_traits<Graph>::vertex_descriptor vertex1,
            typename boost::graph_traits<Graph>::vertex_descriptor vertex2,
            Route& route)
{
    typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_type;
    vertex_type pred = predecessors[vertex2];

    add_edge_to_route(graph, pred, vertex2, route);
    while (pred != vertex1)
    {
        add_edge_to_route(graph, predecessors[pred], pred, route);
        pred = predecessors[pred];
    }
}

#endif