/// @deprecated this file is unmaintained but kept as a reference, remove when appropriate 
///
/// Copyright (c) 2014-2016 Andrew Hundt, Johns Hopkins University
/// 
/// @author Andrew Hundt <ATHundt@gmail.com>
///
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

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

MOVEIT_CLASS_FORWARD( RTreeContext );

class RTreeContext : public planning_interface::PlanningContext {

public:
  
  typedef boost::array<double,6> vertex;
  typedef std::size_t index;
  typedef std::pair<index,index> edge;
  typedef std::vector<index> path;

  RTreeContext( const robot_model::RobotModelConstPtr& model, 
		const std::string &name, 
		const std::string& group, 
		const ros::NodeHandle &nh = ros::NodeHandle("~") );
  virtual ~RTreeContext();

  virtual bool solve( planning_interface::MotionPlanResponse &res );
  virtual bool solve( planning_interface::MotionPlanDetailedResponse &res );

  virtual void clear();
  virtual bool terminate();

  /**
     Feel free to change this method if you can do better.

     Test if a state collides with the scene.
     Call this method if you need to find if the robot collides with the 
     environment in the given robot's state.
     \param[in] q The robot state
     \return      true if the robot state collides. false otherwise
  */
  bool state_collides( const vertex& q ) const;

  /**
     Feel free to change this method if you can do better.

     Test if a linear trajectory between two states collides with the 
     environment. Call this method if you need to find if the robot collides
     with the environment if it moves linearly between two states.
     \param[in] qA   The start robot state
     \param[in] qB   The final robot state
     \param     step The joint step used in between configurations
     \return         true if the robot trajectory collides. false otherwise
  */
  bool edge_collides( const vertex& qA, 
		      const vertex& qB, 
		      double step=0.01 )const;

  /**
     TODO

     Create a vector of N vertices. Each vertex represent a collision-free
     configuration (i.e. 6 joints) of the robot.
     \param N The number of vertices in the graph
     \return  A vector of N collision-free configurations
  */
  std::vector<vertex> make_vertices( int N )const;

template<typename Container>
void make_vertices(Container& nRandomPos, std::size_t N);

  /**
     TODO

     Create a vector of edges. Each vertex represent a pair of vertex indices.
     For example, the pair (1,10) connects the vertices V[1] to V[10]. There is
     no restriction on how many edges your graph requires.
     \param[in] V A vector of N collision-free configurations
     \return  A vector of collision-free edges
  */
  std::vector<edge> make_edges( const std::vector<vertex>& V )const;

  /**
     TODO

     Search for a vertex that is accessible from a start configuration with a 
     collision-free trajectory. The method returns the index of the accessible 
     vertex.
     \param[in] V A vector of N collision-free configurations
     \param[in] q The start configuration
     \return      The index of the accessible vertex
  */
  index search_accessibility( const std::vector<vertex>& V, const vertex& q )const;

  /**
     TODO

     Search for a vertex that is "departable" to a final configuration with a 
     collision-free trajectory. The method returns the index of the departable
     vertex.
     \param[in] V A vector of N collision-free configurations
     \param[in] q The final configuration
     \return      The index of the departable vertex
  */
  index search_departability( const std::vector<vertex>& V, const vertex& q )const;

  /**
     TODO

     Search the graph (V,E) for a path between two vertices.
     collision-free trajectory. The method returns the index of the departable
     vertex.
     \param[in] V          A vector of N collision-free configurations
     \param[in] E          A vector of collision-free edges
     \param[in] access_idx The index of the accessible vertex
     \param[in] depart_idx The index of the departable vertex
     \return               A vector representing a sequence of vertices
  */
  path  search_path( const std::vector<vertex>& V, 
		     const std::vector<edge>& E, 
		     index access_idx, 
		     index depart_idx )const;


protected:

  robot_model::RobotModelConstPtr robotmodel;

};

