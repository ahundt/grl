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

#include <ros/ros.h>

#include "assignment3_context.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>

#include <class_loader/class_loader.h>

#include <dynamic_reconfigure/server.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

class RTreePlanner : public planning_interface::PlannerManager{

public:

  RTreePlanner() : planning_interface::PlannerManager() {}

  virtual 
  bool 
  initialize
  ( const robot_model::RobotModelConstPtr& model, 
    const std::string& ns ){

    if (!ns.empty())
      nh = ros::NodeHandle( ns );
    context.reset( new RTreeContext( model, 
				     std::string( "RTree" ),
				     std::string( "manipulator" ), 
				     nh ) );
    
    return true;

  }

  virtual
  bool 
  canServiceRequest
  ( const moveit_msgs::MotionPlanRequest &req ) const 
  { return true; }

  virtual std::string getDescription() const
  { return std::string( "RTreePlanner" ); }

  virtual
  void 
  getPlanningAlgorithms
  ( std::vector<std::string> &algs ) const{
    algs.resize(1);
    algs[0] = "RTreePlanner";
  }

  virtual 
  planning_interface::PlanningContextPtr 
  getPlanningContext
  ( const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes &error_code) const{
    context->setPlanningScene( planning_scene );
    context->setMotionPlanRequest( req );
    return planning_interface::PlanningContextPtr( context );
  }

  virtual 
  void 
  setPlannerConfigurations
  (const planning_interface::PlannerConfigurationMap &pconfig){}

private:
  
  ros::NodeHandle nh;
  boost::shared_ptr<RTreeContext> context;

};

CLASS_LOADER_REGISTER_CLASS( RTreePlanner, planning_interface::PlannerManager );
