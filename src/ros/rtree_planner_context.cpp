/// @deprecated this file is unmaintained but kept as a reference, remove when appropriate 
///
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


#include "rtree_graph_planner.hpp"
#include "assignment3_context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <boost/foreach.hpp>


// utility function to interpolate between two configuration
RTreeContext::vertex interpolate( const RTreeContext::vertex& qA, 
				  const RTreeContext::vertex& qB, 
				  double t ){  

  RTreeContext::vertex qt;
  boost::fill(qt,0.0);
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;

}

RTreeContext::RTreeContext( const robot_model::RobotModelConstPtr& robotmodel,
			    const std::string& name, 
			    const std::string& group, 
			    const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

RTreeContext::~RTreeContext(){}

bool RTreeContext::state_collides( const vertex& q ) const {

  // create a robot state
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q.begin() );

  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }
  
}

bool RTreeContext::edge_collides( const vertex& qA, 
				  const vertex& qB, 
				  double step )const{
  
  // Simple trajectory in configuration space
  for( double t=0.0; t<1.0; t+=step ){
      if( state_collides( interpolate( qA, qB, t ) ) ) { return true; }
  }

  return false;

}

std::vector<RTreeContext::vertex> RTreeContext::make_vertices( int N )const{

  std::vector<RTreeContext::vertex> V( N );

  for (int i = 0; i < N; ++i) {
    
  }
  // TODO
  // Find and return N vertices that correspond to N collision-free 
  // configurations

  return V;

}

std::vector<RTreeContext::edge> 
RTreeContext::make_edges
( const std::vector<RTreeContext::vertex>& V )const{

  std::vector<RTreeContext::edge> E;

  // TODO
  // Find and return collision-free edges to connect vertices in V

  return E;

}

RTreeContext::index
RTreeContext::search_accessibility
( const std::vector<RTreeContext::vertex>& V, 
  const RTreeContext::vertex& q )const{

  // TODO
  // Find and return the index of the accessible vertex
  // return V.size() if no vertex is accessible

}

RTreeContext::index 
RTreeContext::search_departability
( const std::vector<RTreeContext::vertex>& V, 
  const RTreeContext::vertex& q )const{

  // TODO
  // Find and return the index of the departable vertex
  // return V.size() if no vertex is departable

}


RTreeContext::path 
RTreeContext::search_path
( const std::vector<RTreeContext::vertex>& V,
  const std::vector<RTreeContext::edge>& E,
  RTreeContext::index idx_start,
  RTreeContext::index idx_final )const{

  std::vector<RTreeContext::index> path;

  // TODO
  // Find and return a path between the vertices idx_start and idx_final

  return path;

}

template<typename Container>
void RTreeContext::make_vertices(Container& nRandomPos, std::size_t N) {

    boost::random::mt19937 rng;         // produces randomness out of thin air
                                      // see pseudo-random number generators
    boost::random::uniform_real_distribution<> ptp(-boost::math::constants::pi<double>(),boost::math::constants::pi<double>());
    // distribution that maps -pi to pi
    // see random number distributions
    rng.seed(static_cast<unsigned int>(std::time(0)));
 
    plan::ArmPos nextpos;
  

    // create a robot state
    moveit::core::RobotState robotstate( robotmodel );
    
    for(std::size_t n = 0; n < N; ++n){
        for(std::size_t i = 0; i < plan::ArmPos::size(); ++i){
            nextpos[i] = ptp(rng);       // simulate rolling a die
        }

        robotstate.setJointGroupPositions( "manipulator", nextpos.begin() );

        if(! getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
        { 
             nRandomPos.push_back(nextpos);
        }
       
    }
}
template<typename T,size_t N>
std::ostream& operator<< (std::ostream& out, const boost::array<T,N>& v) {
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "]";
    return out;
}

/// @todo move back to KukaFRIClientDataTest.cpp
template<typename T>
inline std::ostream& operator<<(std::ostream& out,  std::vector<T>& v)
{
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "]";
    return out;
}


// This is the method that is called each time a plan is requested
bool RTreeContext::solve( planning_interface::MotionPlanResponse &res ){

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, 
  							      getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  plan::ArmPos qstart, qfinal;
  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    qfinal[i] = request_.goal_constraints[0].joint_constraints[i].position;
    qstart[i] = request_.start_state.joint_state.position[i];
  }
                   

  // start the timer
  ros::Time begin = ros::Time::now();
  
  // TODO remove temporary line:
  //instantiate_rtree_and_graph();

  // TODO
  // Adjust N to your need
  
  // Create n ramdom points
  int N = 1000;
  int querySize = 10;
  double edgeStep = .05;
  
  std::vector<plan::ArmPos> V;
  V.reserve(N+10);
  make_vertices(V,N);
  
  plan::graph_type graph;
  plan::knn_rtree_type rtree;
  
  std::vector<plan::rtree_value> VtoVd;

  std::cout << "initialized graph and rtree, adding start vertex\n";
  // Add a vertex to the graph
  plan::vertex_descriptor_type qstart_vertex = boost::add_vertex(plan::vertex_property_type(qstart), graph);
  // Add point -> vertex pair to rtree
  plan::rtree_value qstart_vtovd = std::make_pair(qstart,qstart_vertex);
  VtoVd.push_back(qstart_vtovd);
  rtree.insert(qstart_vtovd);

  std::cout << "added start vertex to rtree and graph, adding remaining vertices\n";
  // Add a vertex to the graph
  plan::vertex_descriptor_type qfinal_vertex = boost::add_vertex(plan::vertex_property_type(qfinal), graph);
  // Add point -> vertex pair to rtree
  plan::rtree_value qfinal_vtovd = std::make_pair(qfinal,qfinal_vertex);
  VtoVd.push_back(qfinal_vtovd);
  rtree.insert(qfinal_vtovd);
  
  // put all vertices in the graph and rtree
  BOOST_FOREACH(plan::ArmPos const & apos, V)
  {
      // Add a vertex to the graph
      plan::vertex_descriptor_type new_vertex = boost::add_vertex(plan::vertex_property_type(apos), graph);
      // Add point -> vertex pair to rtree
      plan::rtree_value vtovd = std::make_pair(apos,new_vertex);
      VtoVd.push_back(vtovd);
      rtree.insert(vtovd);
  }

  std::cout << "added all vertices to rtree and graph, polling for valid edges (this takes a while)\n";
  int i = 0;
  int percent = 0;
  
  // get all the nearest neighbors and create a graph
  BOOST_FOREACH(plan::rtree_value const & aposToVertex1, VtoVd)
  {
  
      std::vector<plan::rtree_value> result_n;
      rtree.query(bgi::nearest(aposToVertex1.first, querySize), std::back_inserter(result_n));
      
      BOOST_FOREACH(plan::rtree_value const & aposToVertex2, result_n)
      {
          // add edge if there isn't a problem
          if(!edge_collides(aposToVertex1.first,aposToVertex2.first, edgeStep)) {
              boost::add_edge(
                              aposToVertex1.second,
                              aposToVertex2.second,
                              plan::edge_property_type(plan::line_type(aposToVertex1.first,aposToVertex2.first)),
                              graph
                              );
          }
      }
      
      if(i%(VtoVd.size()/100) == 0){
          std::cout << percent << "\% of edge search complete\n";
          percent++;
      }
      ++i;
  }

  std::cout << "added all edges to graph, running dijkstra\n";

  int const num_v = boost::num_vertices(graph);
  std::vector<plan::vertex_descriptor_type> predecessors(num_v);
  std::vector<double> costs(num_v);        

  // Call Dijkstra (without named-parameter to be compatible with all VC)
  boost::dijkstra_shortest_paths(graph, 
          qfinal_vtovd.second, // find shortest paths to the start
          &predecessors[0], &costs[0],
          boost::get(&plan::edge_property_type::length, graph),
          boost::get(boost::vertex_index, graph),
          std::less<double>(), std::plus<double>(),
          (std::numeric_limits<double>::max)(), double(),
          boost::dijkstra_visitor<boost::null_visitor>());

  std::cout << "dijkstra complete, building route\n";
          
  std::vector<plan::ArmPos> route;
  // build the route from the graph
  build_route(graph,predecessors,qfinal_vtovd.second,qstart_vtovd.second,route);
  // erase consecutive duplicate entries
  std::vector<plan::ArmPos>::iterator it;
  it = std::unique (route.begin(), route.end());
  route.resize( std::distance(route.begin(),it) );

  std::cout << "route complete: \n" << route << "\ncollecting stats and visualizing plan\n";
  // Create a vector of collision-free vertices
  //std::vector<vertex> V = make_vertices( N );

  // // Find the index of the accessible vertex
  // index idx_start = search_accessibility( V, qstart );
  // // Find the index of the departable vertex
  // index idx_final = search_departability( V, qfinal );
  //
  // // Both index must be valid (accessible and departable vertices exist)
  // if( V.size() <= idx_start || V.size() <= idx_final ) { return false; }
  //
  // // Create a vector edges
  // std::vector<edge> E = make_edges( V );
  //
  // // Find a path between the start index and final index
  // path P = search_path( V, E, idx_start, idx_final );

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", &qstart[0] );

// 
//   for( double t=0.0; t<=1.0; t+=0.01 ){
//       /// @todo may need route[1]
//     vertex q = interpolate( qstart, route[0], t );
//     robotstate.setJointGroupPositions( "manipulator", &q[0] );
//     res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
//   }

  for( std::size_t i=0; i<route.size()-1; i++ ){
    for( double t=0.0; t<=1.0; t+=edgeStep ){
      vertex q = interpolate( route[i], route[i+1], t );
      robotstate.setJointGroupPositions( "manipulator", &q[0] );
      res.trajectory_->addSuffixWayPoint( robotstate, edgeStep );
    }
  }

  // for( double t=0.0; t<=1.0; t+=0.01 ){
  //   vertex q = interpolate( V[P[P.size()-1]], qfinal, t );
  //   robotstate.setJointGroupPositions( "manipulator", &q[0] );
  //   res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
  // }

  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
  
}

bool RTreeContext::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void RTreeContext::clear(){}

bool RTreeContext::terminate(){return true;}
