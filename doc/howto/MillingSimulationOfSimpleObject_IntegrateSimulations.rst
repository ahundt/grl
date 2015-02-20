#8 Milling Simulation of simple object
#43 Integrate Simulations
======================================

1. Adding a path: 

	-Right click anywhere on the screen
	-Select add
	-Select Segment Path for a general path.
	Or
	-Select Circle Path to obtain a circle path.

2. Editing a path: 

	-Select your path from the "Scene hierarchy" tab
	-Hit on the "Toggle path edit mode".
	-To change the position or orientation of a point, select it when you are in the "Toggle path edit mode".
	-Hit the "Object/Item Shift" or "Object/Item Rotate" buttons from the tool-bar located at the top of the screen.

3. Following a path: 

	-Right click anywhere on the screen, select add, select "Dummy". You will need two dummies.
	-Make one dummy a child to your robot end_effector link, and the other one a child to your path.
	-Make these two dummies "Linked" by double clicking on one of these dummies icon on the "Scene hierarchy" tab, and selecting the other dummy in the "Linked dummy" option.

You have two options now:

	3.1. Using the "Inverse Kinematics" module:
	
		-Follow the instruction on how to use the "Inverse Kinematics" module from:
		http://www.coppeliarobotics.com/helpFiles/en/inverseKinematicsModule.htm

		-Note that you will need to change the way the robot joints act. Double click on joints' icon in "Scene hierarchy" tab, and change the "Joint Mode" to "Joint in inverse kinematics mode". 
		This will enable the joint to follow the inverse kinematics module. 

	3.2. Using the "Lua" commands from the scene "MillingRobot" and using some of :
		-Functions needed: (Follow the instructions on http://www.coppeliarobotics.com/helpFiles/en/apiFunctionListAlphabetical.htm)
			-SimGetPositionOnPath
			-SimGetOrientationOnPath
			-SimGetObjectPosition
			-SimRMLMovetoPosition
			-SimFollowPath
			-SimGetObjectHandle
			
4. Adding Mill:
 
	-Right click anywhere on the screen, select add, select "Mill" and choose type. Make the mill a child to the dummy which is a child to the robot end_effector.

5. Adding Object: 

	-Right click anywhere on the screen, select add, select "Primitive Shape" 

