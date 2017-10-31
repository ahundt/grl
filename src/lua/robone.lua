--- currently contains useful lua functions for vrep only
--- to use:
--- 1) symlink or copy robone.lua and grl.lua into the same folder as the vrep executable
--- 2) add the following line to your script before you want to use the library:
---       require "grl"
--- 3) call your function! ex: grl.isModuleLoaded('')

robone = {}

require "grl"
------------------------------------------
-- Move the arm along the cut file path --
------------------------------------------
robone.cutBoneScript=function()

	-- require "matrix"

	-- local matrix = require "matrix"
	-- local symbol = matrix.symbol

	-- local matrixJacobian, matrixJacobianTranspose, externalJointTorque, toolTipForces

	-- matrix.add; number
	--m1 = matrix{{8,4,1},{6,8,3}}
	--m2 = matrix{{-8,1,3},{5,2,1}}
	--assert(m1 + m2 == matrix{{0,5,4},{11,10,4}})
	--boneThreadFunction=function()
	--    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
	--
	--    --boneTargetPos = {+0.20,-0.7750,+0.2400}
	--    --boneTargetOri = {0,-90,0}
	--    Path1P,Path1O=grl.getTransformToPathPointInWorldFrame(BoneMovePath,1)
	--    simRMLMoveToPosition(table,-1,-1,nil,nil,maxVel,maxAccel,maxJerk,Path1P,Path1O,nil)
	--
	--    simSwitchThread()
	--    end
	--end

	threadFunction=function()
		-- Retrieves current simulation state. 
		-- http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetSimulationState.htm
		while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
			--Path1P,Path1O=grl.getTransformToPathPointInWorldFrame(BoneMovePath,1)
			--simRMLMoveToPosition(table,-1,-1,nil,nil,maxVel,maxAccel,maxJerk,Path1P,Path1O,nil)
		--boneTargetPos = {+0.20,-0.7750,+0.2400}
		--boneTargetOri = {0,-90,0}

		--Path0P,Path0O=grl.getTransformToPathPointInWorldFrame(BoneMovePath,1)
		--simRMLMoveToPosition(bone,-1,-1,nil,nil,maxVel,maxAccel,maxJerk,Path0P,Path0O,nil)
			--if needToWaitForOpticalTracker then
			--    while os.clock() - starttime <= timeToWaitBeforeStarting do
			--        simSwitchThread()
			--    end
			--end

		--maxVel = 0.04
		--maxForce = 30

		while true do
			if runFollowPathMode then
				-- Moves an object to the position/orientation of another moving object (target object) 
				-- by performing interpolations (i.e. the object will effectiviely follow the target object). 
				simMoveToObject(target,CreatedPathHandle,3,0,0.5,0.02)
				-- http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#simFollowPath
				simFollowPath(target,CreatedPathHandle,3,0,0.01,0.01)
				--simSwitchThread()

			else
				-- Calculate path length
				pathLength = simGetPathLength(CreatedPathHandle)
				-- Move to start of path

				simMoveToObject(target,CreatedPathHandle,3,0,0.2,0.02)
				newPosition = 0



				-- Create empty vector for tool tip forces (x,y,z,alpha,beta,gamma,joint dependence(?))
				--toolTipForces = matrix(7,1,0)
				toolTipForces = {0,0,0,0,0,0,0}
				newPos = simGetPositionOnPath(CreatedPathHandle,0)
				-- While not at end of path, traverse path
				while newPosition < pathLength do
					print("VREP JOINT POSITIONS")
				for i=1,7,1 do
					print(simGetJointPosition(jointHandles[i]))
				end
					externalJointTorque = getExternalJointTorque(measuredJointHandles, externalTorqueHandle)

					measuredForce = calcToolTipForce(IKGroupHandle, externalJointTorque)
					print("MEASURED FORCE: "..measuredForce)

					position = simGetPathPosition(CreatedPathHandle)
					offset = 0.001
					newPosition = position+offset

					simSetPathPosition(CreatedPathHandle, newPosition)
					nextPos = simGetPositionOnPath(CreatedPathHandle, newPosition/pathLength)
					nextEul = simGetOrientationOnPath(CreatedPathHandle, newPosition/pathLength)
					nextQuat = simGetQuaternionFromMatrix(simBuildMatrix({0,0,0},nextEul))
					nextVel = {0,0,0,0}
					for i=1,3,1 do
						nextVel[i] = 3*(nextPos[i]-newPos[i])
					end
			--timeBefore = simGetSimulationTime()
			--print("Time nonRML takes: ".. timeBefore-timeAfter)
			--print("Time before RML: "..timeBefore)
			result,newPos,newQuat,currentVel,currentAccel,timeLeft=simRMLMoveToPosition(target,-1,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,nextPos,nextQuat,nextVel)
			--timeAfter = simGetSimulationTime()
			--print("Time after RML: "..timeAfter)
			--print("Time RML takes: ".. timeAfter-timeBefore)
			--print("Time left: "..timeLeft)


					--fcv(IKGroupHandle, CreatedPathHandle, maxVel, maxForce, measuredForce, pathLength)

					--simSwitchThread()

				end
				simSetPathPosition(CreatedPathHandle, 0)
			end
			end

			--Path0P,Path0O=grl.getTransformToPathPointInWorldFrame(BallJointPath,0)
			--pathPos=simGetObjectPosition(BallJointPath,targetBase)
			--print('pathPos:',pathPos[1],' ',pathPos[2],' ',pathPos[3],' ','\nPath0P: ',Path0P[1],' ',Path0P[2],' ',Path0P[3],' ')

			-- object to approach, frame (world)
			--simRMLMoveToPosition(target,-1,-1,nil,nil,maxVel,maxAccel,maxJerk,Path0P,Path0O,nil)
			-- simFollowPath( objecthandle,pathHandle,position and/or orientation, relative distance on path, velocity, accel

	--        simRMLMoveToPosition(target,-1,-1,nil,nil,maxVel,maxAccel,maxJerk,Path0P,Path0O,nil)


	--        Path0P,Path0O=grl.getTransformToPathPointInWorldFrame(ImplantCutPath,0)
	--        simRMLMoveToPosition(target,-1,-1,nil,nil,maxVel,maxAccel,maxJerk,Path0P,Path0O,nil)
	--      simFollowPath(target,ImplantCutPath,3,0,0.01,0.01)


		end
	end

	-- Initialization:
	simSetThreadSwitchTiming(2)

	--needToWaitForOpticalTracker = true
	--timeToWaitBeforeStarting = 10
	--starttime = os.clock()

	jointHandles={-1,-1,-1,-1,-1,-1,-1}
	measuredJointHandles={-1,-1,-1,-1,-1,-1,-1}
	for i=1,7,1 do
		jointHandles[i]=simGetObjectHandle('LBR_iiwa_14_R820_joint'..i)
		measuredJointHandles[i]=simGetObjectHandle('LBR_iiwa_14_R820_joint'..i..'#0')
	end

	-- Set-up some of the RML vectors:
	--vel=110
	--accel=40
	--jerk=80

	function constrain(arg, min, max)
		if (arg < min) then
			return min
		elseif (arg > max) then
			return max
		else
			return arg
		end
	end

	function forceToOffset(measuredForce, maxForce, maxOffset)
		measuredForce = constrain(measuredForce, 0, maxForce)

		-- Natural log model
		return (maxOffset/math.log(maxForce+1)) * math.log(maxForce + 1 - measuredForce)
		-- Square root model
		--return maxOffset * math.sqrt(1 - measuredForce/maxForce

		--return maxOffset - 0.0004*measuredForce
	end

	function getExternalJointTorque(measuredJointHandles, externalTorqueHandle)
		-- Create empty vector for external joint torques
		externalJointTorque = {0,0,0,0,0,0,0}

		sumOfTorques = 0
		print("TORQUES:")
		for i=1,7,1 do
			externalTorqueData = simGetObjectCustomData(measuredJointHandles[i],externalTorqueHandle)
			if not (externalTorqueData == nil) then
				numberJointTorque = tonumber(externalTorqueData)
				if not (numberJointTorque == nil) then
					externalJointTorque[i] = numberJointTorque
					print(externalJointTorque[i].." ")
					sumOfTorques = sumOfTorques + math.abs(externalJointTorque[i])
				end
			end
		end
		print("\n")
		return externalJointTorque
	end


	function calcToolTipForce(ikGroupHandle, externalTorques)

		if not (simComputeJacobian(ikGroupHandle, 0) == -1) then
			jacobian, jacobianSize = simGetIkGroupMatrix(ikGroupHandle, 0)

			for i=1,3,1 do
				sumOfTransformedTorques = 0;
				for j=1,jacobianSize[1],1 do
					--print("JACOBIAN: "..jacobian[(i-1)*jacobianSize[1]+j])
					sumOfTransformedTorques = sumOfTransformedTorques+jacobian[(i-1)*jacobianSize[1]+j]*externalTorques[j]
				end
				toolTipForces[i] = sumOfTransformedTorques;
			end
			--toolTipForces = matrixJacobianTranspose*externalJointTorque
		end

		return math.sqrt(toolTipForces[1]*toolTipForces[1]+toolTipForces[2]*toolTipForces[2]+toolTipForces[3]*toolTipForces[3])
	end

	function fcv(ikGroupHandle, CreatedPathHandle, maxVelocity, maxForce, measuredForce, pathLength)

		-- selection parameter for simRMLPos for default behavior
		selection = {1,1,1,1,1,1,1}

		-- Get position (in meters) on the path (0 = start, pathLength = end)
		position = simGetPathPosition(CreatedPathHandle)

		--measuredForce = sumOfTorques
		timeStep = simGetSimulationTimeStep()
		maxOffset = maxVelocity*timeStep


		-- Calculate the "speed" (magnitude of position) we wish to move based on force (in meters)
		offset = constrain(forceToOffset(measuredForce, maxForce, maxOffset), 0, maxOffset)
		newPosition = position + offset
		--print("OFFSET: "..maxOffset)

		-- Move intrinsic path position to next position
		simSetPathPosition(CreatedPathHandle, newPosition)

		-- Get the XYZ coordinates of the next position along the path
		nextXyzPosition = simGetPositionOnPath(CreatedPathHandle, newPosition/pathLength)
		nextOrientation = simGetOrientationOnPath(CreatedPathHandle, newPosition/pathLength)

		if not useRMLSmoothing then
			-- Move to the next position on the path
			simSetObjectPosition(target, -1, nextXyzPosition)
			simSetObjectOrientation(target, -1, nextOrientation)

		else
			nextQuat = simGetQuaternionFromMatrix(simBuildMatrix({0,0,0},nextOrientation))
			--simRMLMoveToPosition(target,-1,-1,
		end
	end

	target=simGetObjectHandle('RobotMillTipTarget')
	targetBase=simGetObjectHandle('Robotiiwa')
	bone=simGetObjectHandle('FemurBone')
	table=simGetObjectHandle('highTable')

	testStraightLine = false

	CreatedPathHandle=simGetObjectHandle('MillHipCutPath')

	if testStraightLine then
		CreatedPathHandle=simGetObjectHandle('StraightLinePath')
	end

	BallJointPath=simGetObjectHandle('RemoveBallJoint')
	ImplantCutPath=simGetObjectHandle('ImplantCutPath')
	BoneMovePath=simGetObjectHandle('BonePath')

	IKGroupHandle = simGetIkGroupHandle('IK_Group1_iiwa')

	externalTorqueHandle = 310832412

	-- Get the current position and orientation of the robot's tooltip:
	initPos=simGetObjectPosition(target,targetBase)
	initOr=simGetObjectQuaternion(target,targetBase)

	-- Set-up some of the RML vectors:
	--maxVel={0.4,0.4,0.4,1.8}
	--maxAccel={0.3,0.3,0.3,0.9}
	--maxJerk={0.2,0.2,0.2,0.8}

	maxVel={0.01,0.01,0.01,0.3}
	maxAccel={0.01,0.01,0.01,0.3}
	maxJerk={0.01,0.01,0.01,0.3}

	--activateSuctionPad(false)
	boxIndex=2
	containerIndex=2
	runFollowPathMode = true
	-- Enable/Disable custom IK
	useGrlInverseKinematics=true
	print("useGrlInverseKinematics V-REP plugin: " , useGrlInverseKinematics )

	-- Using RML library for smooth FCV path following
	useRMLSmoothing = false

	if (grl.isModuleLoaded('GrlInverseKinematics') and useGrlInverseKinematics) then
		simExtGrlInverseKinematicsStart()
	end

	print("Moving Robotiiwa arm along cut path...")
	-- Here we execute the regular thread code:
	res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
	if not res then
		simAddStatusbarMessage('Lua runtime error: '..err)
	end
end


------------------------------------------
-- Run Hand Eye Calibration Procedure   --
------------------------------------------
robone.handEyeCalibScript=function()

	jointHandles={-1,-1,-1,-1,-1,-1,-1}
	for i=1,7,1 do
		jointHandles[i]=simGetObjectHandle('LBR_iiwa_14_R820_joint'..i)
	end

	-- Set-up some of the RML vectors:
	vel=7
	accel=2
	jerk=4
	targetV=1
	-- Degrees to Radians
	DtoR=math.pi/180

	currentVel={0,0,0,0}
	currentAccel={0,0,0,0}
	maxVel={vel,vel,vel,vel*DtoR}
	maxAccel={accel,accel,accel,accel*DtoR}
	maxJerk={jerk,jerk,jerk,jerk*DtoR}
	targetVel={targetV,targetV,targetV,targetV*DtoR}

	target=simGetObjectHandle('RobotMillTipTarget')
	-- target=simGetObjectHandle('RobotFlangeTipTarget')
	targetBase=simGetObjectHandle('Robotiiwa')
	path=simGetObjectHandle('HandEyeCalibPath')
	circleCalib = simGetObjectHandle('CircleCalibPath')
	endeffectorTarget=simGetObjectHandle('RobotMillTipTarget')
	numSteps=36

	startP,startO=grl.getTransformBetweenHandles(target,targetBase)

	-- Enable/Disable custom IK
	useGrlInverseKinematics=false

	if (grl.isModuleLoaded('GrlInverseKinematics') and useGrlInverseKinematics) then
		simExtGrlInverseKinematicsStart()
	end
	-- Check if the required plugin is there:
	if (not grl.isModuleLoaded('HandEyeCalibration')) then
		simDisplayDialog('Error','HandEyeCalibration plugin was not found. (v_repExtHandEyeCalibration.dll)&&nSimulation will not run correctly',sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
	else

		-- Run the hand eye calibration

		simExtHandEyeCalibStart()

		for i=0,1,1/numSteps do
			p,o=grl.getPathPointInWorldFrame(path,i)
			simRMLMoveToPosition(target,-1,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,p,o,nil)
			simWait(0.5)
			simExtHandEyeCalibAddFrame()
			simWait(0.25)
		end

		-- move back to start position
		simRMLMoveToPosition(target,targetBase,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,startP,startO,nil)

		-- calculate the transform
		simExtHandEyeCalibFindTransform()
		simExtHandEyeCalibApplyTransform()

	-- check for fusiontrack
	if (not grl.isModuleLoaded('AtracsysFusionTrack')) then
		simDisplayDialog('Error','AtracsysFusionTrack plugin was not found. (v_repExtAtracsysFusionTrack.dll)&&nSimulation will not run correctly',sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
	else
		if(simExtAtracsysFusionTrackIsActive()) then
		-- the sensor won't be in the right position so restore it
		simExtHandEyeCalibRestoreSensorPosition()
		end
	end

	end

end

-----------------------------------------------
-- Initialize real physical KUKA iiwa robot  --
-----------------------------------------------
robone.startRealArmDriverScript=function()

	-- Check if the required plugin is there:
	if (grl.isModuleLoaded('KukaLBRiiwa')) then

		ArmJointNames={

			'LBR_iiwa_14_R820_joint1' , -- Joint1Handle,
			'LBR_iiwa_14_R820_joint2' , -- Joint2Handle,
			'LBR_iiwa_14_R820_joint3' , -- Joint3Handle,
			'LBR_iiwa_14_R820_joint4' , -- Joint4Handle,
			'LBR_iiwa_14_R820_joint5' , -- Joint5Handle,
			'LBR_iiwa_14_R820_joint6' , -- Joint6Handle,
			'LBR_iiwa_14_R820_joint7'   -- Joint7Handle,
		}

		-- Start the physical kuka arm's plugin
		simExtKukaLBRiiwaStart(
			ArmJointNames             , -- list/table of arm joint names
			'RobotFlangeTip'          , -- RobotFlangeTipHandle (aka where tools attach)
			'RobotMillTip'            , -- RobotTipHandle,      (aka tool tip)
			'RobotMillTipTarget'      , -- RobotTargetHandle,   (aka where the tool should go)
			'Robotiiwa'               , -- RobotTargetBaseHandle, (aka robot name)
			'KUKA_LBR_IIWA_14_R820'   , -- RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
			'0.0.0.0'                 , -- LocalUDPAddress  JAVA interface
			'30010'                   , -- LocalUDPPort     JAVA interface
			'172.31.1.147'            , -- RemoteUDPAddress JAVA interface
			'192.170.10.100'          , -- LocalHostKukaKoniUDPAddress,
			'30200'                   , -- LocalHostKukaKoniUDPPort,
			'192.170.10.2'            , -- RemoteHostKukaKoniUDPAddress,
			'30200'                   , -- RemoteHostKukaKoniUDPPort
			'JAVA'                    , -- KukaCommandMode (options are FRI, JAVA)
			'FRI'                     , -- KukaMonitorMode (options are FRI, JAVA)
			"IK_Group1_iiwa"            -- IKGroupName
		)
	else
		simDisplayDialog('Error','KukaLBRiiwa plugin was not found. (v_repExtKukaLBRiiwa.dll)&&nSimulation will run without hardware',sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
	end
end

------------------------------------------
-- Configure the Optical Tracker --
-- Call in a "Customization Script" --
------------------------------------------
robone.configureOpticalTracker=function()

	if (sim_call_type==sim_customizationscriptcall_initialization) then
		-- this is called just after this script was created (or reinitialized)
		-- Do some initialization here

		-- Check if the required plugin is there:

		require "grl"
		simAddStatusbarMessage('robone.configureOpticalTracker() + v_repExtAtracsysFusionTrackVrepPlugin: configuring optical tracker, loading geometry files and defining objects to move')
		-- Check if the required plugin is there:
		if (not grl.isModuleLoaded('AtracsysFusionTrack')) then
			simDisplayDialog('Error','AtracsysFusionTrack plugin was not found. (v_repExtAtracsysFusionTrack.dll)&&nSimulation will not run correctly',sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
		else
			-- define the object in the scene which
			-- represents the coordinate system base of the optical tracker
			simExtAtracsysFusionTrackSetOpticalTrackerBase('OpticalTrackerBase#0')

			-- set the files defining the marker wth balls' geometry
			simAddStatusbarMessage('robone.configureOpticalTracker() + v_repExtAtracsysFusionTrackVrepPlugin: loading geometry0022.ini')
			simExtAtracsysFusionTrackAddGeometry('geometry0022.ini')
			simAddStatusbarMessage('robone.configureOpticalTracker() + v_repExtAtracsysFusionTrackVrepPlugin: loading geometry0055.ini')
			simExtAtracsysFusionTrackAddGeometry('geometry0055.ini')


			--------------------------------------------------
			-- Move the Tracker
			-- true enables moving the tracker, false disables it
			moveTracker = false
			if (moveTracker) then
				simAddStatusbarMessage('robone.configureOpticalTracker() + v_repExtAtracsysFusionTrackVrepPlugin: Moving Optical tracker position relative to marker on robot end effector.')
				simExtAtracsysFusionTrackClearObjects()
				-- The OpticalTrackerBase#0 should move
				-- (base moves relative to Fiducial #22 on the arm)
				simExtAtracsysFusionTrackAddObject('OpticalTrackerBase#0',  -- ObjectToMove
												'Fiducial#22',           -- FrameInWhichToMoveObject
												'Fiducial#22',           -- ObjectBeingMeasured
												'22'                     -- GeometryID
												)
			end

			--------------------------------------------------
			-- Move the Bone
			-- true enables moving the bone, false disables it
			moveBone = not moveTracker
			if (moveBone) then
				simAddStatusbarMessage('robone.configureOpticalTracker() + v_repExtAtracsysFusionTrackVrepPlugin: Moving bone marker position relative to the optical tracker.')
				simExtAtracsysFusionTrackClearObjects()
			-- The bone should move  (bone is attached to Fiducial #55)
				simExtAtracsysFusionTrackAddObject('Fiducial#55',           -- ObjectToMove
												'OpticalTrackerBase#0',  -- FrameInWhichToMoveObject
												'Fiducial#55',           -- ObjectBeingMeasured
												'55'                     -- GeometryID
												)
			end

			-- Start collecting data from the optical tracker
			simExtAtracsysFusionTrackStart()
		end

		-- By default we disable customization script execution during simulation, in order
		-- to run simulations faster:
		simSetScriptAttribute(sim_handle_self,sim_customizationscriptattribute_activeduringsimulation,false)
	end
end


--------------------------------------------------------------------------
-- Get External Joint Torque Data from KUKA iiwa real arm driver plugin --
--------------------------------------------------------------------------
-- Example:
--
-- for i=1,7,1 do
-- 	jointHandles[i]=simGetObjectHandle('LBR_iiwa_14_R820_joint'..i)
-- 	measuredJointHandles[i]=simGetObjectHandle('LBR_iiwa_14_R820_joint'..i..'#0')
-- end
-- externalJointTorque = getExternalJointTorque(7, measuredJointHandles, externalTorqueHandle)
robone.getExternalJointTorque=function(dofs, jointHandles, externalTorqueHandle)
    -- Create empty vector for external joint torques
    externalJointTorque = {}
    --sumOfTorques = 0
    --print("TORQUES:")
    for i=1,dofs,1 do
        externalTorqueData = simGetObjectCustomData(measuredJointHandles[i],externalTorqueHandle)
        if not (externalTorqueData == nil) then
            numberJointTorque = tonumber(externalTorqueData)
            if not (numberJointTorque == nil) then
                externalJointTorque[i] = numberJointTorque
                --print(externalJointTorque[i].." ")
                --sumOfTorques = sumOfTorques + math.abs(externalJointTorque[i])
            end
        end
    end
    --print("\n")
    return externalJointTorque
end

return robone