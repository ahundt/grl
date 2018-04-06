/// @author Andrew Hundt <athundt@gmail.com>
#ifndef _INVERSE_KINEMATICS_VREP_PLUGIN_
#define _INVERSE_KINEMATICS_VREP_PLUGIN_

/// @todo remove IGNORE_ROTATION or make it a runtime configurable parameter
// #define IGNORE_ROTATION

#include <fstream>
#include <vector>
#include <set>
#include <iterator>
#include <algorithm>

#include "flatbuffers/util.h"
#include "grl/flatbuffer/ParseflatbuffertoCSV.hpp"
#include <thirdparty/fbs_tk/fbs_tk.hpp>
#include "grl/flatbuffer/FusionTrack_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/LogKUKAiiwaFusionTrack_generated.h"
#include "grl/time.hpp"


#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Dense>

#include <chrono>
#include <thread>
/// Boost to create an empty folder
#include <boost/filesystem.hpp>


#include <string>
#include <tuple>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// SpaceVecAlg
// https://github.com/jrl-umi3218/SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
// https://github.com/jrl-umi3218/RBDyn
#include <RBDyn/Body.h>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/IK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>

// Tasks
// https://github.com/jrl-umi3218/Tasks
#include <Tasks/Tasks.h>
#include <Tasks/Bounds.h>
#include <Tasks/QPConstr.h>
#include <Tasks/QPContactConstr.h>
#include <Tasks/QPMotionConstr.h>
#include <Tasks/QPSolver.h>
#include <Tasks/QPTasks.h>

// grl
#include "grl/vrep/Vrep.hpp"
#include "grl/vrep/VrepRobotArmDriver.hpp"
#include "grl/vrep/VrepRobotArmJacobian.hpp"
#include "grl/vrep/Eigen.hpp"
#include "grl/vrep/SpaceVecAlg.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/copy.hpp>

#include <spdlog/spdlog.h>

// boost::filesystem
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>


// FusionTrack Libraries
// #include <ftkInterface.h>
// For replay the recorded data
// #include "grl/flatbuffer/ParseflatbuffertoCSV.hpp"
#include <thirdparty/fbs_tk/fbs_tk.hpp>
 #include <boost/type_traits.hpp>

 // mc_rbdyn_urdf
// https://github.com/jrl-umi3218/mc_rbdyn_urdf
#include <mc_rbdyn_urdf/urdf.h>
#include "grl/flatbuffer/kukaiiwaURDF.h"
#include "grl/flatbuffer/CSVIterator.hpp"

#include <cstdlib>

namespace grl {
namespace vrep {

/// Sets the VREP simulation joint positions from the RBDyn configuration
///
/// @param vrepJointNames the name of each joint, order must match vrepJointHandles
/// @param vrepJointHandles the V-REP simulation object handle for each joint
/// @param rbdJointNames the names of all the simArmMultiBody joints, which typically has more joints than the VREP list.
/// @param simArmMultiBody defines the structure of the robot arm for the RBDyn library, see RBDyn documentation
/// @param simArmConfig defines the current position of the robot arm for the RBDyn library, see RBDyn documentation
/// @param debug if empty string, no effect, if any other string a trace of the joint angles will be printed to std::cout
void SetVRepArmFromRBDyn(
    const std::vector<std::string>& vrepJointNames,
    const std::vector<int>& vrepJointHandles,
    const rbd::MultiBody& simArmMultiBody,
    const rbd::MultiBodyConfig& simArmConfig,
    std::string debug = "")
{
    std::string str;
    for (std::size_t i = 0; i < vrepJointHandles.size(); ++ i)
    {
        // modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
        std::string jointName = vrepJointNames[i];
        std::size_t jointIdx = simArmMultiBody.jointIndexByName(jointName);
        float futureAngle = simArmConfig.q[jointIdx][0];
        simSetJointPosition(vrepJointHandles[i],futureAngle);
        /// @todo TODO(ahundt) add torque information
        // simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
        // simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
        // simSetJointTargetPosition(jointHandles_[i],futureAngle);

        if(!debug.empty())
        {
             str+=boost::lexical_cast<std::string>(futureAngle);
             if (i<vrepJointHandles.size()-1) str+=", ";
        }
    }

    if(!debug.empty())
    {
        spdlog::get("console")->info(debug + " jointAngles: " + str);
    }
}


/// Sets the VREP simulation joint positions from the RBDyn configuration
///
/// @param vrepJointNames the name of each joint, order must match vrepJointHandles
/// @param vrepJointHandles the V-REP simulation object handle for each joint
/// @param rbdJointNames the names of all the simArmMultiBody joints, which typically has more joints than the VREP list.
/// @param simArmMultiBody defines the structure of the robot arm for the RBDyn library, see RBDyn documentation
/// @param simArmConfig defines the current position of the robot arm for the RBDyn library, see RBDyn documentation
/// @param debug if empty string, no effect, if any other string a trace of the joint angles will be printed to std::cout
void SetRBDynArmFromVrep(
    const std::vector<std::string>& vrepJointNames,
    const std::vector<int>& vrepJointHandles,
    const rbd::MultiBody& simArmMultiBody,
    rbd::MultiBodyConfig& simArmConfig,
    std::string debug = "")
{
    std::string str;
    float futureAngle;
    for (std::size_t i = 0; i < vrepJointHandles.size(); ++ i)
    {
        // modify parameters as follows https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257466822
        std::string jointName = vrepJointNames[i];
        std::size_t jointIdx = simArmMultiBody.jointIndexByName(jointName);
        simGetJointPosition(vrepJointHandles[i],&futureAngle);
        /// @todo TODO(ahundt) warn/error when size!=0 or explicitly handle those cases
        if(simArmConfig.q[jointIdx].size()>0) simArmConfig.q[jointIdx][0] = futureAngle;

        /// @todo TODO(ahundt) add torque information
        // simSetJointTargetVelocity(jointHandles_[i],jointAngles_dt[i]/simulationTimeStep);
        // simSetJointTargetPosition(jointHandles_[i],jointAngles_dt[i]);
        // simSetJointTargetPosition(jointHandles_[i],futureAngle);

        if(!debug.empty())
        {
             str+=boost::lexical_cast<std::string>(futureAngle);
             if (i<vrepJointHandles.size()-1) str+=", ";
        }
    }

    if(!debug.empty())
    {
        spdlog::get("console")->info(debug + " jointAngles: " + str);
    }
}


/// This object handles taking data from a V-REP based arm simulation
/// using it to configure an arm constrained optimization algorithm,
/// runs the algorithm, then updates the simulation accordingly.
///
/// The types of problems this can solve include reaching a goal position,
/// applying the desired levels of force to a system, and avoiding obstacles
/// at each time step.
///
/// @see the Tasks libraryhttps://github.com/jrl-umi3218/Tasks for a full set of possible capabilities.
///
/// @todo verify Robotiiwa is the correct base, and RobotMillTipTarget is the right target, because if the transform doesn't match the one in the jacobian the algorithm will break
//template<typename DesiredKinematics = DesiredKinematicsObject>
class InverseKinematicsVrepPlugin
{
public:

    //typedef grl::InverseKinematicsController parent_type;
    //using parent_type::currentKinematicsStateP_;
    //using parent_type::parent_type::InitializeKinematicsState;

    enum  VrepVFControllerParamsIndex {
        DesiredKinematicsObjectName,
        IKGroupName
    };

    typedef std::tuple<
             std::string // DesiredKinematicsObjectName
            ,std::string // IKGroupName
        > Params;

    static Params defaultParams()
    {
       return std::make_tuple("RobotMillTipTarget","IK_Group1_iiwa");
    }

    /// @todo need to call parent constructor:
        /*! Constructor
    */
    InverseKinematicsVrepPlugin()
    {
    }

    /// @todo TODO(ahundt) accept params for both simulated and measured arms
    void construct(Params params = defaultParams()){
        // get kinematics group name
        // get number of joints
        logger_ = spdlog::get("console");

        // Get the arm that will be used to generate simulated results to command robot
        // the "base" of this ik is Robotiiwa
        VrepRobotArmDriverSimulatedP_ = std::make_shared<vrep::VrepRobotArmDriver>();
        VrepRobotArmDriverSimulatedP_->construct();
        // Get the "Measured" arm that will be set based off of real arm sensor data, from sources like KukaVrepPlugin or ROS
        // in example simulation this ends in #0
        VrepRobotArmDriverMeasuredP_ = std::make_shared<vrep::VrepRobotArmDriver>(vrep::VrepRobotArmDriver::measuredArmParams());
        VrepRobotArmDriverMeasuredP_->construct();
        measuredJointHandles_ = VrepRobotArmDriverMeasuredP_->getJointHandles();
        measuredJointNames_ = VrepRobotArmDriverMeasuredP_->getJointNames();

        ikGroupHandle_ = simGetIkGroupHandle(std::get<IKGroupName>(params).c_str());
        simSetExplicitHandling(ikGroupHandle_,1); // enable explicit handling for the ik

        auto armDriverSimulatedParams = VrepRobotArmDriverSimulatedP_->getParams();
        // the base frame of the arm
        ikGroupBaseName_ = (std::get<VrepRobotArmDriver::RobotTargetBaseName>(armDriverSimulatedParams));
        // the tip of the arm
        ikGroupTipName_ = (std::get<VrepRobotArmDriver::RobotTipName>(armDriverSimulatedParams));
        // the target, or where the tip of the arm should go
        ikGroupTargetName_ = (std::get<VrepRobotArmDriver::RobotTargetName>(armDriverSimulatedParams));
        robotFlangeTipName_ = (std::get<VrepRobotArmDriver::RobotFlangeTipName>(armDriverSimulatedParams));

        ikGroupBaseHandle_ = grl::vrep::getHandle(ikGroupBaseName_);  // "Robotiiwa"
        ikGroupTipHandle_ = grl::vrep::getHandle(ikGroupTipName_);    // "RobotMillTip"
        ikGroupTargetHandle_ = grl::vrep::getHandle(ikGroupTargetName_);  //  "RobotMillTipTarget"
        /// for why this is named as it is see: https://github.com/jrl-umi3218/Tasks/blob/master/tests/arms.h#L34
        sva::PTransform<double> X_base = getObjectPTransform(ikGroupBaseHandle_);
        //X_base = sva::PTransformd(X_base.rotation().inverse(), X_base.translation());
        // start by assuming the base is fixed
        bool isFixed = true;
        bool isForwardJoint = true;

        { // Initalize the RBDyn arm representation

            jointHandles_ = VrepRobotArmDriverSimulatedP_->getJointHandles();
            jointNames_ = VrepRobotArmDriverSimulatedP_->getJointNames();

            linkNames_ =VrepRobotArmDriverSimulatedP_->getLinkNames();
            linkHandles_ =VrepRobotArmDriverSimulatedP_->getLinkHandles();
            linkRespondableNames_ = VrepRobotArmDriverSimulatedP_->getLinkRespondableNames();
            linkRespondableHandles_ = VrepRobotArmDriverSimulatedP_->getLinkRespondableHandles();

            // save the initial joint angles for later
            // and set the arm to the 0 position for initialization of the
            // optimizer
            std::vector<float> initialJointAngles;
            for (int handle : jointHandles_) {
               float angle;
               simGetJointPosition(handle,&angle);
               simSetJointPosition(handle,0);
               initialJointAngles.push_back(angle);
            }

            rbd_bodyNames_.push_back(ikGroupBaseName_);
            // note: bodyNames are 1 longer than link names, and start with the base!
            boost::copy(linkNames_, std::back_inserter(rbd_bodyNames_));
            boost::copy(jointNames_, std::back_inserter(rbd_jointNames_));
            /// @todo TODO(ahundt) replace hardcoded joint strings with a vrep tree traversal object that generates an RBDyn MultiBodyGraph

            // In this part, it added both joints and links into the jointNames_, why?
            // Should we add them seperately?
            jointNames_.push_back("LBR_iiwa_14_R820_link8");  // Is this a joint? In vrep it's a link.
            jointNames_.push_back("cutter_joint");
            rbd_jointNames_.push_back("LBR_iiwa_14_R820_link8");
            rbd_jointNames_.push_back(robotFlangeTipName_);
            rbd_jointNames_.push_back("cutter_joint");
            rbd_jointNames_.push_back(ikGroupTipName_);  // "RobotMillTip"

            rbd_bodyNames_.push_back("cutter_joint");
            rbd_bodyNames_.push_back(ikGroupTipName_);
            getHandles(rbd_bodyNames_,std::back_inserter(bodyHandles_));
            getHandles(rbd_jointNames_,std::back_inserter(rbd_jointHandles_));

            // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis

            // based on: https://github.com/jrl-umi3218/Tasks/blob/master/tests/arms.h#L34
            // and https://github.com/jrl-umi3218/RBDyn/issues/18#issuecomment-257214536
            for(std::size_t i = 0; i < rbd_jointNames_.size(); i++)
            {

                // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
                std::string thisJointName = rbd_jointNames_[i];
                rbd::Joint::Type jointType = rbd::Joint::Fixed;
                /// @todo TODO(ahundt) fix hard coded Revolute vs fixed joint https://github.com/ahundt/grl/issues/114
                if(simGetObjectType(rbd_jointHandles_[i]) == sim_object_joint_type)
                {
                    jointType = rbd::Joint::Rev;
                }
                rbd::Joint j_i(jointType, Eigen::Vector3d::UnitZ(), isForwardJoint, thisJointName);
                rbd_mbg_.addJoint(j_i);
            }

            // Note that V-REP specifies full transforms to place objects that rotate joints around the Z axis
            /// @todo TODO(ahundt) last "joint" RobotMillTip is really fixed...
            //rbd::Joint j_i(rbd::Joint::Fixed, Eigen::Vector3d::UnitZ(), isForwardJoint, ikGroupTipName_);
            //rbd::Joint j_i(rbd::Joint::Fixed, Eigen::Vector3d::UnitZ(), isForwardJoint, ikGroupTipName_);
            //rbd_mbg_.addJoint(j_i);


            for(std::size_t i = 0; i < rbd_bodyNames_.size(); i++)
            {
                /// @todo TODO(ahundt) extract real inertia and center of mass from V-REP with http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetShapeMassAndInertia.htm

                double mass = 1.;
                auto I = Eigen::Matrix3d::Identity();
                auto h = Eigen::Vector3d::Zero();
                /// @todo TODO(ahundt) consider the origin of the inertia! https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257198604
                /// RBInertia is the Spatial Rigid Body Inertia implementation
                sva::RBInertiad rbi_i(mass, h, I);
                /// @todo TODO(ahundt) add real support for links, particularly the respondable aspects, see LBR_iiwa_14_R820_joint1_resp in RoboneSimulation.ttt
                //  @todo TODO(ahundt) REMEMBER: The first joint is NOT respondable!

                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                std::string bodyName(rbd_bodyNames_[i]);
                rbd::Body b_i(rbi_i,bodyName);

                rbd_mbg_.addBody(b_i);
            }
            // Dummy10 in VRep, by default it's at the origin of the world frame.
            std::string dummyName0(("Dummy"+ boost::lexical_cast<std::string>(0+10)));
            int currentDummy0 = simGetObjectHandle(dummyName0.c_str());
            Eigen::Affine3d eto0 = getObjectTransform(rbd_jointHandles_[0],-1);
            logger_->info("InverseKinematics: \n {} \n{}", dummyName0 , eto0.matrix());
            setObjectTransform(currentDummy0,-1,eto0);

            std::vector<int> frameHandles;  // Joint frames
            frameHandles.push_back(simGetObjectHandle(ikGroupBaseName_.c_str()));
            boost::copy(rbd_jointHandles_,std::back_inserter(frameHandles));
            for(std::size_t i = 0; i < rbd_jointHandles_.size()-1; i++)
            {
                // note: Tasks takes transforms in the successor (child link) frame, so it is the inverse of v-rep
                //       thus we are getting the current joint in the frame of the next joint

                // It seems strange that we get the current joint in the frame of the previous one, not the next one?
                // to means the transform matrix of joint frameHandles[i+1] relative to frameHandles[i]?

                sva::PTransformd to(getObjectPTransform(frameHandles[i+1],frameHandles[i]));
                // this should be the identity matrix because we set the joints to 0!
                //sva::PTransformd from(getJointPTransform(rbd_jointHandles_[i+1]));
                sva::PTransformd from(sva::PTransformd::Identity());

                // Link the PREVIOUSLY created body to the currently created body with the PREVIOUSLY created joint
                // remember, bodyNames[0] is the ikGroupBaseName, so entity order is
                // bodyNames[i], joint[i], bodyNames[i+1]
                std::string prevBody = rbd_bodyNames_[i];
                std::string curJoint = rbd_jointNames_[i];
                std::string nextBody = rbd_bodyNames_[i+1];
                /*  prevBody: Body 1 name;
                    to: Transformation from prevBody to curJoint in prevBody coordinate.
                    nextBody: Body 2 name.
                    from:	Transformation from body 2 to joint in body 2 coordinate.
                    curJoint: Joint between the two body name.
                    isB1toB2 = true:	If true use the joint forward value as body 1 to body 2 forward value of the joint and the inverse value for body 2 to body 1 joint. If false the behavior is inversed.
                //  The body coordinate is fixed at one end of the link, whose origin is coincident with the joint coordinate frame.
                //  Therefore, the to matrix is the transform matrix between two joints and from matrix is Identity().

                */
                rbd_mbg_.linkBodies(prevBody, to, nextBody, from, curJoint);
            }

            
            // Can't understand this comment, since didn't see the inverse of v-rep?
            // note: Tasks takes transforms in the successor (child link) frame, so it is the inverse of v-rep

            rbd_mbs_.push_back(rbd_mbg_.makeMultiBody(ikGroupBaseName_,isFixed,X_base));
            rbd_mbcs_.push_back(rbd::MultiBodyConfig(rbd_mbs_[0]));
            rbd_mbcs_[0].zero(rbd_mbs_[0]);

            // we only have one robot for the moment so the index of it is 0
            const std::size_t simulatedRobotIndex = 0;
            auto& simArmMultiBody = rbd_mbs_[simulatedRobotIndex];
            auto& simArmConfig = rbd_mbcs_[simulatedRobotIndex];

            

            // update the simulated arm position
            for (std::size_t i = 0; i < jointHandles_.size(); ++i) {
               simSetJointPosition(jointHandles_[i],initialJointAngles[i]);
            }

            // Set RBDyn joint angles from vrep joint angles.
            SetRBDynArmFromVrep(jointNames_,jointHandles_,simArmMultiBody,simArmConfig);

            rbd::forwardKinematics(simArmMultiBody, simArmConfig);
            rbd::forwardVelocity(simArmMultiBody, simArmConfig);

            // set the preferred position to the initial position
            // https://github.com/jrl-umi3218/Tasks/blob/15aff94e3e03f6a161a87799ca2cf262b756bd0c/src/QPTasks.h#L426
            rbd_preferred_mbcs_.push_back(simArmConfig);

            debugFrames();

        }

        /// @todo verify object lifetimes

        // add virtual fixtures? need names and number of rows


        /// @todo read objective rows from vrep

        /// @todo set vrep explicit handling of IK here, plus unset in destructor of this object

    }



    /// Set dummy frames named Dummy0 to Dummy19 to the current state of the
    /// RBDyn and V-REP representations of joints for debugging.
    void debugFrames(bool print = false)
    {
        int prevDummy = -1;
        // we only have one robot for the moment so the index of it is 0
        const std::size_t simulatedRobotIndex = 0;
        auto& simArmMultiBody = rbd_mbs_[simulatedRobotIndex];
        auto& simArmConfig = rbd_mbcs_[simulatedRobotIndex];

        // Debug output
        // Every dummy represents a joint.
       for (std::size_t i=0 ; i < jointHandles_.size() ; i++)
       {
          Eigen::Affine3d eto;
          std::string dummyName2(("Dummy"+ boost::lexical_cast<std::string>(i+11)));
          int currentDummy2 = simGetObjectHandle(dummyName2.c_str());
          eto = getObjectTransform(jointHandles_[i],-1);
          setObjectTransform(currentDummy2,-1,eto);
          if(print) logger_->info("{} V-REP World\n{}",dummyName2 , eto.matrix());

          if(i>0)
          {
            Eigen::Affine3d NextJointinPrevFrame(getObjectTransform(jointHandles_[i],jointHandles_[i-1]));
            if(print) logger_->info("{} V-REP JointInPrevFrame\n{}",dummyName2 , NextJointinPrevFrame.matrix());
          }

          bool dummy_world_frame = true;
          // The pose of body i is identical with that of joint i?
          if( dummy_world_frame )
          {
              // visualize each joint position in world frame
              sva::PTransform<double>     plinkWorld = simArmConfig.bodyPosW[simArmMultiBody.bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkWorld = PTranformToEigenAffine(plinkWorld);
              std::string dummyName(("Dummy0"+ boost::lexical_cast<std::string>(i+1)));
              int currentDummy = simGetObjectHandle(dummyName.c_str());
              if(print) logger_->info("{} RBDyn World\n{}",dummyName, linkWorld.matrix());
              setObjectTransform(currentDummy,-1,linkWorld);
              prevDummy=currentDummy;
              // Transformation from parent(i) to i in body coordinate (Xj*Xt).
              sva::PTransform<double>     plinkToSon = simArmConfig.parentToSon[simArmMultiBody.bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkToSon = PTranformToEigenAffine(plinkToSon);
              if(print) logger_->info("{} RBDyn ParentLinkToSon\n{}",dummyName, linkToSon.matrix());

          }
          else
          {
              // visualize each joint position in the previous joint frame

              sva::PTransform<double>     plinkWorld = simArmConfig.parentToSon[simArmMultiBody.bodyIndexByName(linkNames_[i])];
              Eigen::Affine3d linkWorld = PTranformToEigenAffine(plinkWorld);
              std::string dummyName(("Dummy0"+ boost::lexical_cast<std::string>(i+1)));
              int currentDummy = simGetObjectHandle(dummyName.c_str());
              if(print) logger_->info("{} RBDyn\n{}",dummyName , linkWorld.matrix());
              setObjectTransform(currentDummy,prevDummy,linkWorld);
              prevDummy=currentDummy;

          }
       }

       // Frame "Dummy" with no numbers goes at the tip!
       Eigen::Affine3d tipTf = PTranformToEigenAffine(simArmConfig.bodyPosW[simArmMultiBody.bodyIndexByName(ikGroupTipName_)]);
       setObjectTransform(simGetObjectHandle("Dummy"),-1,tipTf);
    }

void getPoseFromCSV(std::string filename, int time_index){
          // we only have one robot for the moment so the index of it is 0
        const std::size_t simulatedRobotIndex = 0;
        auto& simArmMultiBody = rbd_mbs_[simulatedRobotIndex];
        auto& simArmConfig = rbd_mbcs_[simulatedRobotIndex];
        Eigen::VectorXf joint_angles =  Eigen::VectorXf::Zero(7);
        // std::cout << "\nBefore: " << joint_angles  << std::endl;
        int64_t timeStamp = grl::getJointAnglesFromCSV(filename, joint_angles, time_index);
        assert(timeStamp != -1 );

        std::vector<float> vrepJointAngles;
        double angle = 0;
        int jointIdx = 0;
        std::cout << "Time: " << timeStamp  << std::endl;
        for(auto i : jointHandles_)
        {
            angle = joint_angles[jointIdx++];
            simSetJointPosition(i,angle);
            vrepJointAngles.push_back(angle);
            // std::cout << angle << "  ";
        }
        // std::cout << std::endl;

        // Set RBDyn joint angles from vrep joint angles.
        SetRBDynArmFromVrep(jointNames_,jointHandles_,simArmMultiBody,simArmConfig);

        rbd::forwardKinematics(simArmMultiBody, simArmConfig);
        rbd::forwardVelocity(simArmMultiBody, simArmConfig);

        debugFrames();

        std::string opticalTrackerDetectedObjectName("Fiducial#22");
        int opticalTrackerDetectedObjectHandle = simGetObjectHandle(opticalTrackerDetectedObjectName.c_str());
        std::string OpticalTrackerBase("OpticalTrackerBase#0");
        int OpticalTrackerBaseHandle = simGetObjectHandle(OpticalTrackerBase.c_str());
        // get fiducial in optical tracker base frame
        Eigen::Affine3d transformEstimate = getObjectTransform(opticalTrackerDetectedObjectHandle, OpticalTrackerBaseHandle);
        Eigen::VectorXd markerPose = grl::Affine3fToMarkerPose(transformEstimate.cast<float>());
        Eigen::RowVectorXd pose = grl::getPluckerPose(markerPose);

        
        // // Write the hand eye calibration results into a file
        std::string suffix = "ForwardKinematics_Pose.csv";
        auto myFile = boost::filesystem::current_path() /suffix;
        // if(boost::filesystem::exists(myFile)){
        //     boost::filesystem::remove(myFile);
        // }
        boost::filesystem::ofstream ofs(myFile, std::ios_base::app);
        if(time_index == 0) {
            ofs << "local_receive_time_offset_X, K_X,K_Y,K_Z,K_A,K_B,K_C\n";
        }
        ofs << timeStamp << ", " << pose[0] << ", " << pose[1] << ", "<< pose[2] << ", "<< pose[3] << ", "<< pose[4] << ", "<< pose[5] << "\n";
        ofs.close();
   
        
        
       

    }

    void replayMarker(std::string filename, int time_index){
        int rowIdx = time_index*10;
        //for(int i=rowIdx; i<rowIdx+10; ++i){
            auto markerPose = grl::getMarkerPoseFromCSV(filename, time_index);
            Eigen::Vector3d eigenPos(markerPose[0], markerPose[1], markerPose[2]);
            Eigen::Quaternionf eigenQuat;
            eigenQuat = Eigen::AngleAxisf(markerPose[3], Eigen::Vector3f::UnitX())
                      * Eigen::AngleAxisf(markerPose[4], Eigen::Vector3f::UnitY())
                      * Eigen::AngleAxisf(markerPose[5], Eigen::Vector3f::UnitZ());
    
            std::string markerPoseFromTracker("Dummy21");
            int markerPoseHandle = simGetObjectHandle(markerPoseFromTracker.c_str());
    
            std::string OpticalTrackerBase("OpticalTrackerBase#0");
            int OpticalTrackerBaseHandle = simGetObjectHandle(OpticalTrackerBase.c_str());
    
            Eigen::Affine3d transformEstimate = getObjectTransform(markerPoseHandle, OpticalTrackerBaseHandle);
            transformEstimate = eigenQuat.cast<double>();
            transformEstimate.translation() = eigenPos;
    
             // set transform between end effector and fiducial
            setObjectTransform(markerPoseHandle, OpticalTrackerBaseHandle, transformEstimate);
            // logger_->info( "MarkerPose has been set quat wxyz\n: {} , {} , {} ,  {} \n  translation xyz: {}  {}  {}",
            // eigenQuat.w(), eigenQuat.x(), eigenQuat.y(),eigenQuat.z(), transformEstimate.translation().x(), transformEstimate.translation().y(), transformEstimate.translation().z());

        //}

    }


    void testPose(){

       /// simulation tick time step in float seconds from vrep API call
       float simulationTimeStep = simGetSimulationTimeStep();

        // we only have one robot for the moment so the index of it is 0
        const std::size_t simulatedRobotIndex = 0;
        auto& simArmMultiBody = rbd_mbs_[simulatedRobotIndex];
        auto& simArmConfig = rbd_mbcs_[simulatedRobotIndex];
        std::string currentPath = boost::filesystem::current_path().string()+"/data_in/Robone_KukaLBRiiwa.urdf"; 

        auto strRobot = grl::getURDFModel(currentPath);
        rbd::MultiBody mb = strRobot.mb;
        rbd::MultiBodyConfig mbc(mb);
        rbd::MultiBodyGraph mbg(strRobot.mbg);
        std::size_t nrJoints = mb.nrJoints();
        std::size_t nrBodies = strRobot.mb.nrBodies();
        std::vector<std::string> jointNames;
        
        std::vector<float> vrepJointAngles;
        double angle = 0.7;
        for(auto i : jointHandles_)
        {
            angle *= -1;
            simSetJointPosition(i,angle);
            vrepJointAngles.push_back(angle);
        }

      

       /// @todo TODO(ahundt) rethink where/when/how to send command for the joint angles. Return to LUA? Set Directly? Distribute via vrep send message command?

        ////////////////////////////////////////////////////
        // Set joints to current arm position in simulation
        SetRBDynArmFromVrep(jointNames_,jointHandles_,simArmMultiBody,simArmConfig);
        rbd::forwardKinematics(simArmMultiBody, simArmConfig);
        rbd::forwardVelocity(simArmMultiBody, simArmConfig);

        // SetRBDynArmFromVrep(jointNames_,jointHandles_,mb,mbc);
        // rbd::forwardKinematics(mb, mbc);
        // rbd::forwardVelocity(mb, mbc);

        debugFrames();
    }



  
    /// Configures updateKinematics with the goal the kinematics should aim for
    enum class GoalPosE { realGoalPosition, debugGoalPosition };
    /// Configures updateKinematics the algorithm the kinematics should use for solving
    enum class AlgToUseE { ik, multiIterQP, singleIterQP };
    /// Configures if the target pose objective should stick to the constant initalized
    /// version, or if it should update every iteration in tasks::qp::PostureTask
    /// https://github.com/jrl-umi3218/Tasks/blob/15aff94e3e03f6a161a87799ca2cf262b756bd0c/src/QPTasks.h#L426
    enum class PostureTaskStrategyE { constant, updateToCurrent };
    /// Determines where the current arm state is copied from
    /// simulatedArm is the current simulation, measuredArm
    /// is based off of data measured from the real physical arm
    enum class ArmStateSource { simulatedArm, measuredArm };


    /// Runs inverse kinematics or constrained optimization at every simulation time step
    /// @param runOnce Set runOnce = true to only update kinematics once for debugging purposes. runOnce = false runs this function at every time step.
    /// @param solveForPosition defines where the arm is aiming for, the moving goal pose object or a stationary debug position to verify the algorithm
    /// @param postureStrategy helps determine the position/orientation at which the elbow is held
    /// @param syncSimulatedKinematicsState data used to set the simulated arm position at the START of the kinematics update, use measuredArm with the real arm, simulatedArm when running simulation only.
    /// @param syncMeasuredKinematicsState set measured arm position at END of the kinematics update. Use simulatedArm when running simulation, measuredArm when running with a real physical arm from which you can get measurements.
    /// @param scaleSimulationTimeStep scale the amount of time the Tasks library thinks is passing. WARNING: DANGEROUS HACK, MAY WORKS AROUND SPEED LIMITS, MIGHT WANT TO LEAVE THIS AT 1.0
    void updateKinematics(
        const bool runOnce = false,
        const GoalPosE solveForPosition = GoalPosE::realGoalPosition,
        const AlgToUseE alg = AlgToUseE::multiIterQP,
        const PostureTaskStrategyE postureStrategy = PostureTaskStrategyE::constant,
        const ArmStateSource syncSimulatedKinematicsState = ArmStateSource::measuredArm,
        const ArmStateSource syncMeasuredKinematicsState = ArmStateSource::measuredArm,
        int numSolverIterations = 10,
        double scaleSimulationTimeStep = 10
    ){
        if(runOnce && ranOnce_) return;
        ranOnce_ = true;

        jointHandles_ = VrepRobotArmDriverSimulatedP_->getJointHandles();


        ///////////////////////////////////////////////////////////
        // Copy Joint Interval, the range of motion for each joint

        /// @todo TODO(ahundt) change source of current state based on if physical arm is running
        if(syncSimulatedKinematicsState == ArmStateSource::simulatedArm)
        {
            VrepRobotArmDriverSimulatedP_->getState(currentArmState_);
        }
        else
        {
            VrepRobotArmDriverMeasuredP_->getState(currentArmState_);
        }


        // lower limits
        auto & llim = std::get<vrep::VrepRobotArmDriver::JointLowerPositionLimit>(currentArmState_);
        std::vector<double> llimits(llim.begin(),llim.end());

        // upper limits
        auto & ulim = std::get<vrep::VrepRobotArmDriver::JointUpperPositionLimit>(currentArmState_);
        std::vector<double> ulimits(ulim.begin(),ulim.end());

        // current position
        auto & currentJointPos = std::get<vrep::VrepRobotArmDriver::JointPosition>(currentArmState_);
        std::vector<double> currentJointPosVec(currentJointPos.begin(),currentJointPos.end());

        /// @todo TODO(ahundt) also copy acceleration velocity, torque, etc

        // Get the current and desired transform in V-Rep
        const auto& handleParams = VrepRobotArmDriverSimulatedP_->getVrepHandleParams();
        Eigen::Affine3d currentEndEffectorPose =
        getObjectTransform(
                             std::get<vrep::VrepRobotArmDriver::RobotTipName>(handleParams)
                            ,std::get<vrep::VrepRobotArmDriver::RobotTargetBaseName>(handleParams)
                          );
        auto  currentEigenT = currentEndEffectorPose.translation();


        Eigen::Affine3d desiredEndEffectorPose =
        getObjectTransform(
                             std::get<vrep::VrepRobotArmDriver::RobotTargetName>(handleParams)
                            ,std::get<vrep::VrepRobotArmDriver::RobotTargetBaseName>(handleParams)
                          );
        auto  desiredEigenT = desiredEndEffectorPose.translation();

        //////////////////////
        /// @todo TODO(ahundt) move code below here back into separate independent setup and solve functions, move some steps like limits to construct()

       /// simulation tick time step in float seconds from vrep API call
       float simulationTimeStep = simGetSimulationTimeStep();

        // we only have one robot for the moment so the index of it is 0
        const std::size_t simulatedRobotIndex = 0;
        auto& simArmMultiBody = rbd_mbs_[simulatedRobotIndex];
        auto& simArmConfig = rbd_mbcs_[simulatedRobotIndex];



       /// @todo TODO(ahundt) rethink where/when/how to send command for the joint angles. Return to LUA? Set Directly? Distribute via vrep send message command?

        ////////////////////////////////////////////////////
        // Set joints to current arm position in simulation
        if(syncSimulatedKinematicsState == ArmStateSource::simulatedArm)
        {
            SetRBDynArmFromVrep(jointNames_,jointHandles_,simArmMultiBody,simArmConfig);
        }
        else
        {
            // note that the regular joint names are used but the measured handles are used
            // this means the names will match correctly for accessing the multibody, while
            // measured angles are what will actually be transferred.
            /// @todo TODO(ahundt) using it like this is a bit fragile, and a reasonable change to
            ///                    SetRBDynArmFromVrep could cause breakage, so improve the API design.
            SetRBDynArmFromVrep(jointNames_,measuredJointHandles_,simArmMultiBody,simArmConfig);
        }
        rbd::forwardKinematics(simArmMultiBody, simArmConfig);
        rbd::forwardVelocity(simArmMultiBody, simArmConfig);

        // save the current MultiBodyConfig for comparison after running algorithms
        rbd_prev_mbcs_ = rbd_mbcs_;
        // set the preferred position to the current position
        // https://github.com/jrl-umi3218/Tasks/blob/15aff94e3e03f6a161a87799ca2cf262b756bd0c/src/QPTasks.h#L426
        if(postureStrategy == PostureTaskStrategyE::updateToCurrent) rbd_preferred_mbcs_ = rbd_mbcs_;

        /// @todo TODO(ahundt) make solver object a member variable if possible, initialize in constructor
        tasks::qp::QPSolver solver;

        ////////////////////////////////////////////////////
        // Set position goal of the arm
        sva::PTransformd targetWorldTransform;

        if( solveForPosition == GoalPosE::realGoalPosition )
        {
            // go to the real target position
            targetWorldTransform = getObjectPTransform(ikGroupTargetHandle_);
            // logger_->info("target translation (vrep format):\n{}", targetWorldTransform.translation());
        }
        else
        {
            // go to a debugging target position
            targetWorldTransform = simArmConfig.bodyPosW[simArmMultiBody.bodyIndexByName(ikGroupTipName_)];
            targetWorldTransform.translation().z() -= 0.0001;
            // logger_->info("target translation (rbdyn format):\n", targetWorldTransform.translation());
        }
        tasks::qp::PositionTask posTask(rbd_mbs_, simulatedRobotIndex, ikGroupTipName_,targetWorldTransform.translation());
        tasks::qp::SetPointTask posTaskSp(rbd_mbs_, simulatedRobotIndex, &posTask, 50., 10.);
        tasks::qp::OrientationTask oriTask(rbd_mbs_,simulatedRobotIndex, ikGroupTipName_,targetWorldTransform.rotation());
        tasks::qp::SetPointTask oriTaskSp(rbd_mbs_, simulatedRobotIndex, &oriTask, 50., 1.);
        tasks::qp::PostureTask postureTask(rbd_mbs_,simulatedRobotIndex,rbd_preferred_mbcs_[simulatedRobotIndex].q,1,0.01);

        double inf = std::numeric_limits<double>::infinity();


        ////////////////////////////////////
        // Set joint limits

        // joint limit objects, initialize to current q so entries
        // will be reasonable, such as empty entries for fixed joints
        std::vector<std::vector<double> > lBound = simArmConfig.q;
        std::vector<std::vector<double> > uBound = simArmConfig.q;
        std::vector<std::vector<double> > lVelBound = simArmConfig.alpha;
        std::vector<std::vector<double> > uVelBound = simArmConfig.alpha;


        // for all joints
        for (std::size_t i=0 ; i < rbd_jointHandles_.size()-1; i++)
        {
            /// limits must be organized as described in https://github.com/jrl-umi3218/Tasks/issues/10#issuecomment-257793242
            std::string jointName = rbd_jointNames_[i];
            std::size_t jointIdx = simArmMultiBody.jointIndexByName(jointName);

            /// @todo TODO(ahundt) ulimits aren't the same size as jointHandles_, need velocity limits too
            // set joint position limits
            if(boost::iequals(jointName,"cutter_joint"))
            { /// @todo TODO(ahundt) hardcoded mill tip joint limits, remove these, load from simulation
                lBound[jointIdx][0] = -inf;
                uBound[jointIdx][0] = inf;
                lVelBound[jointIdx][0] = -inf;
                uVelBound[jointIdx][0] = inf;
            }
            else if(i<llimits.size() && lBound[jointIdx].size()==1)
            {
                lBound[jointIdx][0] = llimits[i];
                uBound[jointIdx][0] = ulimits[i];
            }

            // set joint velocity limits
            if(i<llimits.size() && lVelBound[jointIdx].size()==1)
            {
                /// @todo TODO(ahundt) replace hardcoded velocity limits with real and useful limits specific to each joint loaded from V-REP or another file.
                // 0.99 radians per second is the limit of the slowest joint (joint 1) on the KUKA iiwa R820
                lVelBound[jointIdx][0] = -0.99;
                uVelBound[jointIdx][0] = 0.99;
            }
        }

        tasks::qp::DamperJointLimitsConstr dampJointConstr(rbd_mbs_, simulatedRobotIndex, {lBound, uBound},{lVelBound, uVelBound}, 0.125, 0.025, 1., simulationTimeStep);

        // Test add*Constraint
        dampJointConstr.addToSolver(solver);
        BOOST_VERIFY(solver.nrBoundConstraints() == 1);
        BOOST_VERIFY(solver.nrConstraints() == 1);

        solver.nrVars(rbd_mbs_, {}, {});
        solver.updateConstrSize();

        solver.addTask(&posTaskSp);
        BOOST_VERIFY(solver.nrTasks() == 1);
        solver.addTask(&oriTaskSp);
        solver.addTask(&postureTask);

        ////////////////////////////////////
        // Run constrained optimization
        if(alg == AlgToUseE::ik)
        {
            // use basic inverse kinematics to solve for the position
            //rbd::InverseKinematics ik(simArmMultiBody,simArmMultiBody.jointIndexByName(jointNames_[6]));
            rbd::InverseKinematics ik(simArmMultiBody,simArmMultiBody.bodyIndexByName(ikGroupTipName_));
            // Update the simArmMultiBody,simArmConfig based on the inverse kinematics.
            ik.sInverseKinematics(simArmMultiBody,simArmConfig,targetWorldTransform);
            // update the simulated arm position
        }
        else if( alg == AlgToUseE::multiIterQP)
        {
            // multiple iteration version of solving
            // Test JointLimitsConstr
            /// @todo TODO(ahundt) was this commented correctly?
            //simArmConfig = mbcInit;
            double timeStepDividedIntoIterations = scaleSimulationTimeStep*simulationTimeStep/numSolverIterations;
            // This actually runs every time step, so only one iteration here, unless we want to subdivide
            // a v-rep time step into smaller rbdyn time steps.
            for(int i = 0; i < numSolverIterations; ++i)
            {
                //BOOST_REQUIRE(solver.solve(rbd_mbs_, rbd_mbcs_));
                /// @todo TODO(ahundt) remove BOOST_VERIFY(), try/catch solver failure, print an error, send a v-rep message, and shut down solver or enter a hand guiding mode
                BOOST_VERIFY(solver.solve(rbd_mbs_, rbd_mbcs_));
                // This should be handled by the simulator or physical robot, "forward simulation of dynamics"
                rbd::sEulerIntegration(simArmMultiBody, simArmConfig, timeStepDividedIntoIterations);
                //rbd::sEulerIntegration(simArmMultiBody, simArmConfig, simulationTimeStep);

                rbd::sForwardKinematics(simArmMultiBody, simArmConfig);
                rbd::sForwardVelocity(simArmMultiBody, simArmConfig);
                //BOOST_REQUIRE_GT(simArmConfig.q[1][simulatedRobotIndex], -cst::pi<double>()/4. - 0.01);
            }
        }
        else
        {
            // single iteration version of solving
            BOOST_VERIFY(solver.solve(rbd_mbs_, rbd_mbcs_));
            rbd::sEulerIntegration(simArmMultiBody, simArmConfig, simulationTimeStep);
            rbd::sForwardKinematics(simArmMultiBody, simArmConfig);
            rbd::sForwardVelocity(simArmMultiBody, simArmConfig);
            // update the simulated arm position
        }

        SetVRepArmFromRBDyn(jointNames_,jointHandles_,simArmMultiBody,simArmConfig);


        if(syncMeasuredKinematicsState == ArmStateSource::simulatedArm)
        {
            // Running simulation only, so update the "measured" arm to match the simulated arm

            // note that the regular joint names are used but the measured handles are used
            // this means the names will match correctly for accessing the multibody, while
            // measured angles are what will actually be transferred.
            /// @todo TODO(ahundt) using it like this is a bit fragile, and a reasonable change to
            ///                    SetRBDynArmFromVrep could cause breakage, so improve the API design.
            SetVRepArmFromRBDyn(jointNames_,measuredJointHandles_,simArmMultiBody,simArmConfig);
        }

       debugFrames();
    } // end updateKinematics()

    /// may not need this it is in the base class
    /// blocking call, call in separate thread, just allocates memory
    void run_one(){
       // If true, run real inverse kinematics algorith, if false go to a test pose.
       const bool ik = false;
       if(ik) {
           updateKinematics();
       } else {
           auto myFile = boost::filesystem::current_path() /"data_in";
           std::string pathName = myFile.string();
           std::string kukaJoint = pathName+"/KUKA_Joint.csv";   // KUKA_Joint.csv   FTKUKA_TimeEvent.csv
           std::string markerPose = pathName+"/FT_Pose_Marker22.csv";
           
           testPose();
           // getPoseFromCSV(kukaJoint, time_index);
           // replayMarker(markerPose, time_index);
           // time_index++;
       }
    }


    /// may not need this it is in the base class
    /// this will have output
    /// blocking call, call in separate thread, just allocates memory
    /// this runs the actual optimization algorithm
    void solve(){

    }
    /// for the replay purpose.
    int time_index = 0;
    fbs_tk::Root<grl::flatbuffer::KUKAiiwaStates> kukaStatesP_;

    std::shared_ptr<spdlog::logger> logger_;

    rbd::MultiBodyGraph                 rbd_mbg_;
    std::vector<rbd::MultiBody>         rbd_mbs_;
    std::vector<rbd::MultiBodyConfig>   rbd_mbcs_;
    /// rbd_prev_mbcs_ is for debugging
    std::vector<rbd::MultiBodyConfig>   rbd_prev_mbcs_;
    /// preferred posture to resolve ambiguities
    std::vector<rbd::MultiBodyConfig>   rbd_preferred_mbcs_;
    std::vector<rbd::Body>              rbd_bodies_;
    std::vector<sva::RBInertia<double>> rbd_inertias_;
    std::vector<rbd::Joint>             rbd_joints_;
    std::vector<std::string>            rbd_bodyNames_;
    std::vector<int>                    bodyHandles_;
    /// rbd "joints" include fixed joints that bridge
    /// various v-rep objects like the world origin and the ik group base.
    /// This is organized a bit differently from the v-rep joint names
    /// because we need additional fixed "joints" so we have transforms
    /// that match the V-REP scene
    std::vector<std::string>            rbd_jointNames_;
    /// rbd "joints" include fixed joints that bridge
    /// various v-rep objects like the world origin and the ik group base.
    /// This is organized a bit differently from the v-rep joint names
    /// because we need additional fixed "joints" so we have transforms
    /// that match the V-REP scene
    std::vector<int>                    rbd_jointHandles_;


    //tasks::qp::QPSolver qp_solver_;

    std::vector<int> jointHandles_; ///< @todo move this item back into VrepRobotArmDriver
    std::vector<int> linkHandles_;
    std::vector<int> linkRespondableHandles_;
    std::vector<std::string> jointNames_;
    std::vector<std::string> linkNames_;
    std::vector<std::string> linkRespondableNames_;


    std::vector<int> measuredJointHandles_; ///< @todo move this item back into VrepRobotArmDriver
    std::vector<std::string> measuredJointNames_;

    int ikGroupHandle_;
    int ikGroupBaseHandle_;
    int ikGroupTipHandle_;
    int ikGroupTargetHandle_;

    std::string ikGroupBaseName_;
    std::string ikGroupTipName_;
    std::string ikGroupTargetName_;
    std::string robotFlangeTipName_; // not part of the V-REP ik group

    std::shared_ptr<vrep::VrepRobotArmDriver> VrepRobotArmDriverSimulatedP_;
    std::shared_ptr<vrep::VrepRobotArmDriver> VrepRobotArmDriverMeasuredP_;
    vrep::VrepRobotArmDriver::State currentArmState_;

    bool ranOnce_ = false;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace vrep
} // namespace grl

#endif
