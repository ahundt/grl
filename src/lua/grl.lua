--- currently contains useful lua functions for vrep only
--- to use: 
--- 1) symlink or copy grl.lua into the same folder as the vrep executable
--- 2) add the following line to your script before you want to use the library:
---       require "grl"
--- 3) call your function! ex: grl.isModuleLoaded('')

grl = {}

grl.isModuleLoaded=function(moduleName)
    -- Check if the required plugin is there:
    local loadedModuleName=0
    local moduleVersion=0
    local index=0
    local isModuleFound=false
    while loadedModuleName do
    	loadedModuleName,moduleVersion=simGetModuleName(index)
    	if (loadedModuleName==moduleName) then
    		isModuleFound=true
    	end
    	index=index+1
    end
    return isModuleFound
end

grl.getQuaternionFromEulerAngle=function(position,eulerAngle)
	local m=simBuildMatrix(position,eulerAngle)
	local q=simGetQuaternionFromMatrix(m)
	return q
end

--- Get the pose of one object relative to another
---
--- @param objectpositionHandle The Object you want to get the position of
--- @param relativeToBaseFrameHandle The object or frame the position should be defined in, nil for world
--- @return position,orientation a 3 vector and quaternion value pair
grl.getTransformBetweenHandles=function(objectPositionHandle, relativeToBaseFrameHandle)
	local p=simGetObjectPosition(objectPositionHandle,relativeToBaseFrameHandle)
	local o=simGetObjectQuaternion(objectPositionHandle,relativeToBaseFrameHandle)
	return p,o
end


--- Get the pose of a point on a path in the world frame
---
--- @param pathHandle handle representing the path object you are referring to
--- @param relative distance along a path from the start at 0 to the end at 1 (percentage*100)
--- @return position,orientation a 3 vector and quaternion value pair
grl.getPathPointInWorldFrame=function(pathHandle,relativeDistance)
	local p=simGetPositionOnPath(pathHandle,relativeDistance)
    local euler_o=simGetOrientationOnPath(pathHandle,relativeDistance)
	local o=grl.getQuaternionFromEulerAngle(p,euler_o)
	return p,o
end

--- @brief Load a vrep path from a file
---
--- @note if the objectName parameter already exists, it will default to the first of path,path0,path1 which doesn't exist.
---
--- @todo can be 11 or 16 elements, read the file and determine it as you go
---
--- @param fileName path and name of file to load, for example 'pathFile.csv'
--- @param objectName vrep object name you can use to get the handle later, for example 'MyPath', see simSetObjectName
--- @param pathProperties http://www.coppeliarobotics.com/helpFiles/en/apiConstants.htm#pathObjectProperties
---
--- @return handle to the path
---
--- @see path overview http://www.coppeliarobotics.com/helpFiles/en/paths.htm
--- @see path import/export http://www.coppeliarobotics.com/helpFiles/en/pathImportExport.htm
grl.loadPathFile=function(fileName, objectName, pathProperties)
    if(pathProperties == nil) then
        pathProperties = sim_pathproperty_show_line or sim_pathproperty_show_orientation or sim_pathproperty_infinite_acceleration or sim_pathproperty_show_position
    end

	local file = io.open(fileName, "r")
	local ctrlPointsBufferTable = {}	--define buffer table

	local lineCount = 0;					--csv line counter
    local lineElementCount=11;
    
    for line in io.lines(fileName) do
        if(lineCount==0) then
            lineCopy=line
            -- string.gsub returns the number of replacements
            -- each line will have 1 less comma then values
            -- @see https://stackoverflow.com/questions/11152220/counting-number-of-string-occurrences
            local _,lineElementCount = string.gsub(lineCopy,",",",")
            lineElementCount = lineElementCount+1 -- 1 more value than commas
        end 
    	lineCount = lineCount + 1
	end
    
    --Number of data in csv file(each line contains 11 data, or 16 data with aux values)
	local fileTotalElementCount = lineCount * lineElementCount		


	for n =1,fileTotalElementCount do
	  ctrlPointsBufferTable[n] =file:read('*n'); file:read(1)
	end
	
    --- @see http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#simInsertPathCtrlPoints
    --- set the flag indicating if there are 11 or 16 elements in each row of the csv file
    local pathCtrlPointsOptions = 0
    if(lineElementCount == 16) then
        pathCtrlPointsOptions = 2 -- see LuaScriptFunctions.cpp
    end 
	--point1 = {0.324854,-0.825006,0.215077,-0,0,math.pi,1.000000,0,15,0.500000,0.500000} --Orientation should be in radian
	--point2 = {0.424805,-0.750010,0.215113,-0.000000,0.000000,math.pi,1.000000,0,15,0.5,0.5}

	--BallJointPath=simGetObjectHandle('RemoveBallJoint')
	--pathPosition=simGetObjectPosition(BallJointPath,-1)
	--pathOrientation=simGetObjectOrientation(BallJointPath,-1)

	local MyPathHandle = simCreatePath(pathProperties,nil,nil,nil)
	--simSetObjectPosition(MyPathHandle,-1,pathPosition)
	--simSetObjectOrientation(MyPathHandle,-1,pathOrientation)

	simInsertPathCtrlPoints(MyPathHandle, pathCtrlPointsOptions, 0, lineCount, ctrlPointsBufferTable)
    local initialName=simGetObjectName(MyPathHandle)
    local initialHandle=simGetObjectHandle(initialName)
	simSetObjectName(initialHandle,objectName)
	--simGetObjectPosition()

	io.close(file)
    
    return MyPathHandle
end

grl.setPoseRelativeToParent=function(objectHandle,parentHandle,objectPositionXYZ,objectOrientationRPY)
	simSetObjectPosition(objectHandle,parentHandle,objectPositionXYZ)
	simSetObjectOrientation(objectHandle,parentHandle,objectOrientationRPY)
	simSetObjectParent(objectHandle,parentHandle,true)
end

return grl