package grl.driver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import grl.ProcessDataManager;
import grl.StartStopSwitchUI;
import grl.TeachMode;
import grl.UpdateConfiguration;
import grl.ZMQManager;
import grl.flatbuffer.ArmState;
import grl.flatbuffer.CartesianImpedenceControlMode;
import grl.flatbuffer.KUKAiiwaInterface;
import grl.flatbuffer.KUKAiiwaMonitorState;
import grl.flatbuffer.KUKAiiwaState;
import grl.flatbuffer.KUKAiiwaStates;
import grl.flatbuffer.MoveArmJointServo;
import grl.flatbuffer.MoveArmTrajectory;
import grl.flatbuffer.Wrench;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.google.flatbuffers.FlatBufferBuilder;
import com.google.flatbuffers.Table;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.ServoMotionJP;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
//import com.kuka.generated.ioAccess.FlexFellowIOGroup;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.recovery.IRecovery;
import com.kuka.roboticsAPI.deviceModel.JointLimits;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.executionModel.IExecutionContainer;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainerListener;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.HandGuidingControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;

/**
 * Creates a FRI Session.
 */
public class GRL_Driver_ChangeModes extends RoboticsAPIApplication
{
	private ProcessDataManager _processDataManager = null; // Stores variables that can be modified by teach pendant in "Process Data" Menu
	private Controller _lbrController;
	private LBR _lbr;
	private StartStopSwitchUI _startStopUI = null;
	
	/// The interface on which commands are being sent,
	/// includes SmartServo, DirectServo, and FRI
	private byte _lbrInterface = KUKAiiwaInterface.SmartServo;
	
	private FRIConfiguration _friConfiguration = null;
	private FRISession       _friSession = null;
	
	private SmartServo         _smartServoMotion = null;
	private ISmartServoRuntime _smartServoRuntime = null;
	
//	//second smart servo to allow changing mode type
//	private SmartServo         _secondSmartServoMotion = null;
//	private ISmartServoRuntime _secondSmartServoRuntime = null;
	
	private grl.flatbuffer.KUKAiiwaState _currentKUKAiiwaState = null;
	private grl.flatbuffer.KUKAiiwaState _previousKUKAiiwaState = null;
	private AbstractMotionControlMode _activeMotionControlMode;
	private AbstractMotionControlMode _smartServoMotionControlMode;
	private UpdateConfiguration _updateConfiguration;
	private IRecovery _pausedApplicationRecovery = null;
	private PhysicalObject _toolAttachedToLBR;
	private HandGuidingMotion _handGuidingMotion;
	/**
	 *  gripper, tool or other physically attached object
	 *  see "Template Data" panel in top right pane
	 *  of Sunrise Workbench. This can't be created
	 *  at runtime so we create one for you.
	 */
	private Tool    _flangeAttachment;
	private JointLimits _jointLimits;
	private double[] _maxAllowedJointLimits;
	private double[] _minAllowedJointLimits;
	private JointImpedanceControlMode _teachControlMode;
	private TeachMode _teachModeRunnable = null;
	private Thread _teachModeThread = null;
	//boolean isModeChanged = false;

//	private MediaFlangeIOGroup _mediaFlangeIOGroup; // this is an end effector flange with a blue ring and buttons on the arm tip
//	private boolean _flexFellowPresent = false; // this is a white an orange robot cart
//	private boolean _mediaFlangeIOPresent = true;
	
	private boolean _canServo = true;
	// when we receive a message
	int message_counter = 0;
	int message_counter_since_last_mode_change = 0;
    boolean waitingForUserToEndTeachMode = false;
    boolean waitingForSwitchMode = false ;
    
    private Lock configureSmartServoLock = new ReentrantLock();
    
    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 40;
    
    final boolean debugPrintoutFlag = false;
    
    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };
    
   // private FlexFellowIOGroup _flexFellowIOGroup;
	
	@Override
	public void initialize()
	{
		_startStopUI = new StartStopSwitchUI(this);
		_processDataManager = new ProcessDataManager(this);
		_lbrController = (Controller) getContext().getControllers().toArray()[0];
		_lbr = (LBR) _lbrController.getDevices().toArray()[0];
		
		//if(_flexFellowPresent) _flexFellowIOGroup = new FlexFellowIOGroup(_lbrController);
//		if(_mediaFlangeIOPresent) {
//			_mediaFlangeIOGroup = new MediaFlangeIOGroup(_lbrController);
//		//	_mediaFlangeIOGroup.setLEDBlue(true);
//		}

        _friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _processDataManager.get_FRI_KONI_LaptopIPAddress());
        _friConfiguration.setSendPeriodMilliSec(4);

        _friSession = new FRISession(_friConfiguration);

		_flangeAttachment = getApplicationData().createFromTemplate("UltrasoundCamera");
		_pausedApplicationRecovery = getRecovery();

		LoadData _loadData = new LoadData();
		_loadData.setMass(_processDataManager.getEndEffectorWeight());
		_loadData.setCenterOfMass(_processDataManager.getEndEffectorX(),
				_processDataManager.getEndEffectorY(),
				_processDataManager.getEndEffectorZ());
		//_toolAttachedToLBR = new Tool("Tool", _loadData);
		// TODO: make configurable via C++ API?
		_toolAttachedToLBR = _flangeAttachment;
		_toolAttachedToLBR.attachTo(_lbr.getFlange());

		_jointLimits = _lbr.getJointLimits();
		
		// used when setting limits in _HandGuidingMotion
		_maxAllowedJointLimits = _jointLimits.getMaxJointPosition().get();
		_minAllowedJointLimits = _jointLimits.getMinJointPosition().get();
		for (int i = 0; i < _lbr.getJointCount(); i++) {
			_maxAllowedJointLimits[i] -= 0.05;
			_minAllowedJointLimits[i] += 0.05;
		}


		_teachControlMode = new JointImpedanceControlMode(_lbr.getJointCount())
								.setStiffnessForAllJoints(0.1)
								.setDampingForAllJoints(0.7);
	

		_teachModeRunnable = new TeachMode(
				_flangeAttachment,
				_maxAllowedJointLimits,
				_minAllowedJointLimits);
		_teachModeRunnable.setLogger(getLogger());
		_teachModeThread = new Thread(_teachModeRunnable);
		_teachModeThread.start();
	}
	

	@Override
	public void run()
	{

		getLogger().info("GRL_Driver from github.com/ahundt/grl starting...\nZMQ Connecting to: " + _processDataManager.get_ZMQ_MASTER_URI());

		// connect to the controlling application via ZeroMQ
		ZMQManager zmq = new ZMQManager(_processDataManager.get_ZMQ_MASTER_URI(),getLogger());
		
		// if stop is ever set to true the program stops running and exits
		boolean stop = zmq.connect();
		IMotionContainer currentMotion = null;

		boolean newConfig = false;

		// TODO: Let user set mode (teach/joint control from tablet as a backup!)
		//this.getApplicationData().getProcessData("DefaultMode").
		
		
		// TODO: add a message that we send to the driver with data log strings
		while (!stop && !_startStopUI.is_stopped()) {
			message_counter+=1;
			_currentKUKAiiwaState = zmq.waitForNextMessage();
			_previousKUKAiiwaState = zmq.getPrevMessage();

			// Print out a notice when switching modes
			if( message_counter == 1 || (_previousKUKAiiwaState != null && _previousKUKAiiwaState.armControlState() != null &&
					_currentKUKAiiwaState.armControlState().stateType() != _previousKUKAiiwaState.armControlState().stateType()))
			{
				getLogger()
				.info("Switching mode: "
						+ ArmState.name(_currentKUKAiiwaState.armControlState().stateType()));
				message_counter_since_last_mode_change  = 0;
			} else {

				message_counter_since_last_mode_change++;
			}

			if(_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.ShutdownArm){
				///////////////////////////////////////////////
				// ShutdownArm
				getLogger()
				.info("ShutdownArm command received, stopping GRL_Driver...");
				stop = true;
			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.TeachArm) {
				///////////////////////////////////////////////
				///////////////////////////////////////////////
				// TeachArm teach mode
				///////////////////////////////////////////////
				///////////////////////////////////////////////

				if(message_counter!=1 && (_previousKUKAiiwaState == null || _previousKUKAiiwaState.armControlState()==null)) {
					continue;
				}
				else if(message_counter==1  
						|| _currentKUKAiiwaState.armControlState().stateType()!=_previousKUKAiiwaState.armControlState().stateType()
						|| (currentMotion != null && currentMotion.isFinished()) ) {
					// Teach mode, changing from some other mode

					if (currentMotion != null) {
						if(!currentMotion.isFinished()) {
							currentMotion.cancel();
						} else {
							getLogger().info("Hand Guiding Motion has finished!");
						}
					}
					
                    if(!cancelSmartServo()) continue;

					getLogger().warn("Enabling Teach Mode with gravity compensation. mode id code = " +
							_currentKUKAiiwaState.armControlState().stateType());
					
					//RK: changed to false because we do not have button like Andrew robot
					boolean useHandGuidingMotion = false;
					
					if(useHandGuidingMotion)
					{
						
						_teachModeRunnable.enable();
							
						getLogger().warn("Started teach thread!");
					}
					else
					{
						currentMotion = _flangeAttachment.moveAsync(positionHold(_teachControlMode, -1, TimeUnit.SECONDS));
					}
				}
				
			}
			else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.MoveArmTrajectory) {
				///////////////////////////////////////////////
				// MoveArmTrajectory mode (sequence of joint angles)
				///////////////////////////////////////////////
                getLogger().error("MoveArmTrajectory Not Yet Implemented!");

				// create an JointPosition Instance, to play with
				JointPosition destination = new JointPosition(
						_lbr.getJointCount());
				// TODO: not fully implemented
				if(!cancelSmartServo()) continue;
				if(!cancelTeachMode()) continue;
                
				if (currentMotion != null) currentMotion.cancel();
                


				MoveArmTrajectory mat;
				if(_currentKUKAiiwaState.armControlState() != null) 
				{
					mat = (MoveArmTrajectory)_currentKUKAiiwaState.armControlState().state(new MoveArmTrajectory());
				} else {
					getLogger().error("Received null armControlState in servo!");
					continue;
				}
				
				for (int j = 0; j < mat.trajLength(); j++)
				{

					JointPosition pos = new JointPosition(_lbr.getCurrentJointPosition());

					for (int k = 0; k < destination.getAxisCount(); ++k)
					{
						//destination.set(k, maj.traj(j).position(k));
						pos.set(k, mat.traj(j).position(k));
						currentMotion = _lbr.moveAsync(ptp(pos));
					}

				}
			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.MoveArmJointServo) {
				///////////////////////////////////////////////
				///////////////////////////////////////////////
				// MoveArmJointServo mode
				///////////////////////////////////////////////
				///////////////////////////////////////////////
				
				if(!cancelTeachMode()) continue;
                
				if (currentMotion != null){
					currentMotion.cancel();
					}
				
                
				// create an JointPosition Instance, to play with
				JointPosition destination = new JointPosition(_lbr.getCurrentJointPosition());	
				
				if(_currentKUKAiiwaState.armControlState() == null) {
					getLogger().error("Received null armControlState in servo!");
					continue;
                    // return;
				}
                
				MoveArmJointServo mas = (MoveArmJointServo)_currentKUKAiiwaState.armControlState().state(new MoveArmJointServo());
				if(_smartServoMotion == null)
				{
					 getLogger().info("At the begining of the cycle. Starting smart servo for the first time.\n");
					 
					 configureSmartServoLock.lock();
					 destination = _lbr.getCurrentJointPosition();
					 
					 if( _smartServoMotionControlMode == null )
					 {
						 getLogger().info("At the begining of the cycle. Smart Servo motion cotnrol mode is null.\n");
				        	_smartServoMotionControlMode = getMotionControlMode(grl.flatbuffer.EControlMode.CART_IMP_CONTROL_MODE);
					 }
					 getLogger().info("Setting up new smart servo.\n");
					 		_smartServoMotion = startNewSmartServo(_smartServoMotionControlMode,destination);
					 configureSmartServoLock.unlock();		
					
					
				}
				
				if (_smartServoRuntime == null) {
					getLogger().info("Setting up Smart Servo runtime");
					try {
//						currentMotion.cancel();
						// TODO: make motion control mode configurable over zmq interface
						configureSmartServoLock.lock();
						ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
						_smartServoRuntime = _smartServoMotion.getRuntime();
						ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
						getLogger().info("Time stamp of update with Real time system: " + _smartServoRuntime.getTimeStampOfUpdate() + "/n");
						configureSmartServoLock.unlock();
						
						 getLogger().info("After: _smartServoMotion.getRuntime().\n");
				        _activeMotionControlMode = _smartServoMotionControlMode;
					} catch (java.lang.IllegalStateException ex) {
						getLogger().error("Could not retrieve SmartServo runtime!");
						
						getLogger().error(ex.getMessage());
						ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
						_smartServoRuntime = null;
						_smartServoMotion = null;
						continue;

					}
				}
				if((_smartServoRuntime != null) )  {
					try {
						if((message_counter_since_last_mode_change > 0))
						{
							// getLogger().error("I am in the recieve destination from grl\n");
							grl.flatbuffer.JointState jointState = mas.goal();
							if(jointState.positionLength()!=destination.getAxisCount()){
								if(message_counter_since_last_mode_change % 500 == 0) getLogger().error("Didn't receive correct number of joints! skipping to start of loop...\n");
								destination = _lbr.getCurrentJointPosition();
								continue;
							}
							else
							{
								//getLogger().error("Getting position from grl!!\n");
								if(!waitingForSwitchMode)
								{
									
									String pos = "pos:";
									for (int k = 0; k < destination.getAxisCount(); ++k)
									{
										double position = jointState.position(k); //Math.round(jointState.position(k)*100.0)/100.0;
										destination.set(k, position);
									    pos = pos + " " + k + ": " + position;
								}
									
									if(message_counter % 500 == 0) getLogger().info("Sample of new Smart Servo Joint destination " + pos);
//									 getLogger().info("New Position called");
									if(_lbr.isReadyToMove())
									{
										_smartServoRuntime.setDestination(destination);

									}
								}
								else
								{
									destination = _lbr.getCurrentJointPosition();
									if(_lbr.isReadyToMove())
									{
										_smartServoRuntime.setDestination(destination);
									}
									waitingForSwitchMode = false;
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
								}
							}
						}
						else
						{
							destination = _lbr.getCurrentJointPosition();
							_smartServoRuntime.waitForTransferred();
							
							if(_lbr.isReadyToMove())
							{
								_smartServoRuntime.setDestination(destination);
							}
						
						}
						// RISTO Start
						if(_currentKUKAiiwaState.armConfiguration().setCartImpedance().isCartImpedanceSet())
						{
							_activeMotionControlMode = getMotionControlMode(grl.flatbuffer.EControlMode.CART_IMP_CONTROL_MODE);
							getLogger().info("Change Cartesian Impedance Settings");
							if(sameControlMode(_activeMotionControlMode,_smartServoMotionControlMode))
							{	
								getLogger().info("Change Settings Cartesian Impedance: " + _activeMotionControlMode.toString());
								_smartServoRuntime.changeControlModeSettings(_activeMotionControlMode);
							}
							else{		
								getLogger().info("Different Control Modes. Stop SmartServo and start new one Cartesian Impedance Smart Servo.");
								getLogger().info("Active CTRL Mode: " + _activeMotionControlMode.toString() + "; Smart servo motion CTRL Mode: " + _smartServoMotionControlMode.toString());
								configureSmartServoLock.lock();
								waitingForSwitchMode = true;
								destination = _lbr.getCurrentJointPosition();
								getLogger()
								.info("Switching mode: "
										+ ArmState.name(_currentKUKAiiwaState.armControlState().stateType()));
								message_counter_since_last_mode_change  = 0;
							
								getLogger().info("before turn off cart f/t control: 0: " + destination.get(0) + ", 1: " + destination.get(0));
								if(cancelSmartServo() ){ //  && destination.equals(_lbr.getCurrentJointPosition())
									getLogger().info("SmartServo is stopped!! Starting new Smart Servo for Cartesian Impedance");
									
									/////////////////////////////////////////////////////////////////////
									grl.flatbuffer.JointState jointState = mas.goal();
									if(jointState.positionLength()!=destination.getAxisCount()){
										if(message_counter_since_last_mode_change % 500 == 0) getLogger().error("Didn't receive correct number of joints! skipping to start of loop...\n");
										destination = _lbr.getCurrentJointPosition();
									}
									else
									{	
										getLogger().info("CartImpedance: Change modes, setting recieved destination.");
											String pos = "pos:";
											for (int k = 0; k < destination.getAxisCount(); ++k)
											{
												double position = jointState.position(k); //Math.round(jointState.position(k)*100.0)/100.0;
												destination.set(k, position);
											    pos = pos + " " + k + ": " + position;
											}
											
									}	
								/////////////////////////////////////////////////////////////////////
									_smartServoMotion = startNewSmartServo(_activeMotionControlMode,destination);
									if(_smartServoRuntime == null)
									{
										getLogger().info("In change to Cartesian Impedance Smart Servo. Smart servo Runtime is null");
										   try {
												// TODO: make motion control mode configurable over zmq interface
											   
											   //TODO: on the second run here we have an error. smart servo runtime does not getting transferred
												_smartServoRuntime = _smartServoMotion.getRuntime();
												_smartServoMotionControlMode = _activeMotionControlMode;
												getLogger().info("In change to Cartesian Impedance Smart Servo. Smart servo Runtime is: " + _smartServoRuntime.toString());
												ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
												//_smartServoRuntime.waitForTransferred();
												getLogger().info("After wait to transfer. Is smart servo now null: " + _smartServoRuntime.toString());
											} catch (java.lang.IllegalStateException ex) {
												getLogger().error("Could not retrieve SmartServo runtime!");
												_smartServoRuntime = null;
												_smartServoMotion = null;
												getLogger().error(ex.getMessage());
												continue;
											}	
									}
									else
									{
										getLogger().info("SmartServo Runtime is not null!!");
									}
									//_secondSmartServoMotion = null;
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
									_smartServoRuntime.updateWithRealtimeSystem();
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
									destination = _smartServoRuntime.getAxisQMsrOnController();
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
			
								}
								else{
									getLogger().info("In Cart Impedance: Waiting smart servo to stop and all motions to finish.");
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
							
								}
								//message_counter_since_last_mode_change++;
								_smartServoRuntime.waitForTransferred();
								ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
								configureSmartServoLock.unlock(); 

								
							}
						}
						if(_currentKUKAiiwaState.armConfiguration().setConstCartForce().isConstFTSet())
						{
							getLogger().info("Setting up Const Force/Torque Cartesian Control");
							_activeMotionControlMode = getMotionControlMode(grl.flatbuffer.EControlMode.CART_FORCE_CONTROL_MODE);
						
							if(sameControlMode(_activeMotionControlMode,_smartServoMotionControlMode))
							{
			
								getLogger().info("Change Settings Cartesian F/T: " + _activeMotionControlMode.toString());
								_smartServoRuntime.changeControlModeSettings(_activeMotionControlMode);
							}
							else
							{
								
								getLogger().info("Different Control Modes. Stop SmartServo and start new one FT Smart Servo");
								getLogger().info("Active CTRL Mode: " + _activeMotionControlMode.toString() + "smart servo motion CTRL Mode: " + _smartServoMotionControlMode.toString());
								waitingForSwitchMode = true;
								getLogger()
								.info("Switching mode: "
										+ ArmState.name(_currentKUKAiiwaState.armControlState().stateType()));
								message_counter_since_last_mode_change  = 0;
//								destination = ;
								//_smartServoRuntime.setDestination(_lbr.getCurrentJointPosition());
								
								configureSmartServoLock.lock();
								destination = _lbr.getCurrentJointPosition();
								getLogger().info("before turn off cart impedance: 0: " + destination.get(0) + ", 1: " + destination.get(1));
								if(cancelSmartServo()){ // && destination.equals(_lbr.getCurrentJointPosition())
									getLogger().info("SmartServo is stopped!! Starting new Smart Servo for FT");
									/////////////////////////////////////////////////////////////////////
									grl.flatbuffer.JointState jointState = mas.goal();
									if(jointState.positionLength()!=destination.getAxisCount()){
										if(message_counter_since_last_mode_change % 500 == 0) getLogger().error("Didn't receive correct number of joints! skipping to start of loop...\n");
										destination = _lbr.getCurrentJointPosition();
									}
									else
									{
										getLogger().info("F/T Ctrl: Change modes, setting recieved destination.");
											String pos = "pos:";
											for (int k = 0; k < destination.getAxisCount(); ++k)
											{
												double position = jointState.position(k); //Math.round(jointState.position(k)*100.0)/100.0;
												destination.set(k, position);
											    pos = pos + " " + k + ": " + position;
											}
											//_smartServoRuntime.updateWithRealtimeSystem();
									}	
								/////////////////////////////////////////////////////////////////////
									_smartServoMotion = startNewSmartServo(_activeMotionControlMode,destination);
									if(_smartServoRuntime == null)
									{
										getLogger().info("In change to Cartesian F/T Smart Servo. Smart servo Runtime is null");
										   try {
												// TODO: make motion control mode configurable over zmq interface
												_smartServoRuntime = _smartServoMotion.getRuntime();
												_smartServoMotionControlMode = _activeMotionControlMode;
												getLogger().info("In change to Cartesian F/T Smart Servo. Smart servo Runtime is: " + _smartServoRuntime.toString());
												ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
//												_smartServoRuntime.updateWithRealtimeSystem();
												
											} catch (java.lang.IllegalStateException ex) {
												getLogger().error("Could not retrieve SmartServo runtime!");
												_smartServoRuntime = null;
												_smartServoMotion = null;
												getLogger().error(ex.getMessage());
												continue;
												//return;
											}	
									}
									else
									{
										getLogger().info("SmartServo Runtime is not null!!");
									}
									
									//_secondSmartServoMotion = null;
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
									_smartServoRuntime.updateWithRealtimeSystem();
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
									destination = _smartServoRuntime.getAxisQMsrOnController();
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
								 //   _smartServoRuntime.setDestination(destination);
//									while (!_smartServoRuntime.isDestinationReached())
//							        {
//							            ThreadUtil.milliSleep((long) (MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT + _smartServoRuntime.getRemainingTime()));
//							            _smartServoRuntime.updateWithRealtimeSystem();
//							            getLogger().info("in FT2: Updated with real time system done!!");
//							        }
								    
								}
								else{
									getLogger().info("In FT: Waiting smart servo to stop.");
									ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
									continue;
								}
								//message_counter_since_last_mode_change++;
								_smartServoRuntime.waitForTransferred();
								ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
								configureSmartServoLock.unlock();
							}
								
						}
	// RISTO END
					} catch (CommandInvalidException e) {
						getLogger().error("Could not update smart servo destination! Clearing SmartServo.");
//						_smartServoRuntime.stopMotion();
						destination =_lbr.getCurrentJointPosition();
						_smartServoRuntime.stopMotion();
						_smartServoMotion = null;
						_smartServoRuntime = null;
						continue;
					} catch (java.lang.IllegalStateException ex) {
						getLogger().error("Could not update smart servo destination and missed the real exception type! Clearing SmartServo.");
//						_smartServoRuntime.stopMotion();
						destination =_lbr.getCurrentJointPosition();
						_smartServoRuntime.stopMotion();
						_smartServoMotion = null;
						_smartServoRuntime = null;
						getLogger().error(ex.getMessage());
						continue;
					}

			   } else {
					getLogger().error("Couldn't issue motion command, smartServo motion was most likely reset. retrying...");
					//_smartServoRuntime.stopMotion();
			   }

			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.StopArm) {

				_smartServoRuntime.stopMotion();
				if (currentMotion != null) {
					currentMotion.cancel();
				}
				if(!cancelTeachMode()) continue;
				
				//tm.setActive(false);
			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.PauseArm) {
		
				if (currentMotion != null) currentMotion.cancel();
				if(!cancelTeachMode()) continue;
				if(!cancelSmartServo()) continue;
				
				PositionControlMode controlMode = new PositionControlMode();
				if(message_counter_since_last_mode_change < 2 || message_counter_since_last_mode_change % 100 == 0){
					_lbr.move(positionHold(controlMode,10,TimeUnit.MILLISECONDS));
				}

			} else {
				System.out.println("Unsupported Mode! stopping");
				stop = true;
			}
//		// Force feedback	
//			if(message_counter % 1000000 == 0) 
//			{
//		//	getLogger().info("Sample of new Smart Servo Joint destination message sent");
//
//			
//			/// Reading sensor values from Java Interface and sending them thrugh ZMQ
//			if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.MoveArmJointServo){
//				
//			
//			double force_x = _lbr.getExternalForceTorque(_lbr.getFlange()).getForce().getX();
//			double force_y = _lbr.getExternalForceTorque(_lbr.getFlange()).getForce().getY();
//			double force_z = _lbr.getExternalForceTorque(_lbr.getFlange()).getForce().getZ();	
//			
//			double torque_x = _lbr.getExternalForceTorque(_lbr.getFlange()).getTorque().getX();	
//			double torque_y = _lbr.getExternalForceTorque(_lbr.getFlange()).getTorque().getY();
//			double torque_z = _lbr.getExternalForceTorque(_lbr.getFlange()).getTorque().getZ();
//				
//		
//			FlatBufferBuilder builder = new FlatBufferBuilder(0);
//			
//			int fb_wrench = Wrench.createWrench(builder, force_x, force_y, force_z, torque_x, torque_y, torque_z, 0, 0, 0);
//		
//			KUKAiiwaMonitorState.startKUKAiiwaMonitorState(builder);
//		    KUKAiiwaMonitorState.addCartesianWrench(builder, fb_wrench);
//			int monitorStateOffset = KUKAiiwaMonitorState.endKUKAiiwaMonitorState(builder);
//			
//			KUKAiiwaState.startKUKAiiwaState(builder);
//			KUKAiiwaState.addMonitorState(builder, monitorStateOffset);
//			int[] statesOffset = new int[1];
//			statesOffset[0] = KUKAiiwaState.endKUKAiiwaState(builder);
//			
//			int statesVector = KUKAiiwaStates.createStatesVector(builder, statesOffset);
//			
//			KUKAiiwaStates.startKUKAiiwaStates(builder);
//			KUKAiiwaStates.addStates(builder, statesVector);
//			int KUKAiiwaStatesOffset = KUKAiiwaStates.endKUKAiiwaStates(builder);
//			
//			
//			builder.finish(KUKAiiwaStatesOffset);
//			byte[] msg = builder.sizedByteArray();
//		
//		//	zmq.sendMessage(msg);
//			
//		//	KUKAiiwaStates myStates = KUKAiiwaStates.getRootAsKUKAiiwaStates(builder.dataBuffer());
//		//	double z = myStates.states(0).monitorState().CartesianWrench().force().z();
//		//	getLogger().info("force_z " + z + "\n");
//			
////			try {
////				Thread.sleep(10);
////			} catch (InterruptedException e) {
////				// TODO Auto-generated catch block
////				e.printStackTrace();
////			}
//			
//			
//		//	getLogger().info("lenght: " + msg.length + "\n");
//			}
//			}
		} // end primary while loop


		// done
		zmq.stop();
		_teachModeRunnable.stop();
		if (_updateConfiguration!=null && _updateConfiguration.get_FRISession() != null) {
			_updateConfiguration.get_FRISession().close();
		}
		
		getLogger()
		.info("ZMQ connection closed.\nExiting...\nThanks for using the\nGRL_Driver from github.com/ahundt/grl\nGoodbye!");
		//System.exit(1);
	}

	/**
	 * 
	 * @return true if teach mode completed successfully; false if we are waiting on a user to press a physical button
	 */
	private boolean cancelTeachMode() {
		if (_teachModeThread != null) {
			//getLogger().warn("Cancelling teach mode...");
			_canServo = _teachModeRunnable.cancel();
//			if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightYellow(false);

			if(!_teachModeRunnable.isEnableEnded() && 
				_currentKUKAiiwaState.armControlState().stateType() != grl.flatbuffer.ArmState.TeachArm){

				if(message_counter % 30 == 0) getLogger().warn("Can't Exit Teach Mode Yet,\n Did You Press The Hand Guiding Button?");
                waitingForUserToEndTeachMode = true;
//				if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightRed(true);
				return false;
			}
            else if(_teachModeRunnable.isEnableEnded()
                    && _currentKUKAiiwaState.armControlState().stateType() != grl.flatbuffer.ArmState.TeachArm
                    && waitingForUserToEndTeachMode)// && waitingForUserToEndTeachMode
            {
					PositionControlMode controlMode = new PositionControlMode();
					_lbr.move(positionHold(controlMode,10,TimeUnit.MILLISECONDS));
                    waitingForUserToEndTeachMode = false;
//        			if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightRed(false);
            }
		}
		return true;
	}
	/**
	 * 
	 * @return true if teach mode completed successfully; false if something went wrong
	 */
    private boolean cancelSmartServo(){
		if (_smartServoRuntime != null) {
			_smartServoRuntime.stopMotion();
			_smartServoMotion = null;
			_smartServoRuntime = null;
//			if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightGreen(false);
			getLogger().info("Ending smart servo!");
		}
        return true;
    }
    private SmartServo startNewSmartServo(AbstractMotionControlMode controlMode, JointPosition startDestination)
    {
    	// create an JointPosition Instance, to play with
//		JointPosition destination = new JointPosition(_lbr.getJointCount());

//		destination = new JointPosition(_lbr.getCurrentJointPosition());
    	
//        final JointPosition initialPosition = new JointPosition(
//                _lbr.getCurrentJointPosition());
        
        
		
        SmartServo _ssmot = new SmartServo(startDestination);
        
		String pos = "pos:";
		for (int k = 0; k < startDestination.getAxisCount(); ++k)
		{
			double position = startDestination.get(k);
		    pos = pos + " " + k + ": " + position;
		}
		getLogger().info("New Smart Servo Initial Position " + pos);
		
    	/*
         * 
         * Note: The Validation itself justifies, that in this very time
         * instance, the load parameter setting was sufficient. This does not
         * mean by far, that the parameter setting is valid in the sequel or
         * lifetime of this program
         */
        try
        {
            if (!ServoMotion.validateForImpedanceMode(_flangeAttachment))
            {
                getLogger().info("Validation of torque model failed - correct your mass property settings");
                getLogger().info("SmartServo will be available for position controlled mode only, until validation is performed");
            }
//            else
//            {
//            	_ssmot.setMode(controlMode);
//            }
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
        
        // Set the motion properties to % of system abilities. For example .2 is 20% of systems abilities
        // TODO: load these over C++ interface
        // Set the motion properties to 10% of the systems abilities
        _ssmot.setJointAccelerationRel(0.1);
        _ssmot.setJointVelocityRel(0.1);
        _ssmot.setMinimumTrajectoryExecutionTime(5e-3);
        
//        _ssmot
//        	.setSpeedTimeoutAfterGoalReach(0.1);
//        	.setJointAccelerationRel(_processDataManager.get_jointAccelRel())
//        	.setJointVelocityRel(_processDataManager.get_jointVelRel())
//        	.setMinimumTrajectoryExecutionTime(20e-3);


        _ssmot.setMode(controlMode);
        getLogger().info("Before setting MoveASync in new Smart Servo!!");
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(_ssmot);
        //_ssmot.getRuntime().updateWithRealtimeSystem();
        getLogger().info("SmartServo Control Mode is Changed");
    	
    	return _ssmot;
    	
    }
	boolean sameControlMode(AbstractMotionControlMode changecm, AbstractMotionControlMode currentcm)
	{
		String currentcmString = changecm.getClass().getSimpleName();
		String changedccmString = currentcm.getClass().getSimpleName();
		
		getLogger().info("Control Mode are equal: " + currentcmString.equals(changedccmString));
		return currentcmString.equals(changedccmString);
	}
	/**
	 * Initialize the appropriate control mode based on passed parameters
	 * 
	 * @param controlMode grl.flatbuffer.EControlMode
	 * @return
	 */
	public AbstractMotionControlMode getMotionControlMode(byte controlMode)
	{
		AbstractMotionControlMode mcm = null;

		if(controlMode == grl.flatbuffer.EControlMode.POSITION_CONTROL_MODE){
			mcm = new PositionControlMode();
		} else if(controlMode==grl.flatbuffer.EControlMode.CART_IMP_CONTROL_MODE){
			
	
	        if(_currentKUKAiiwaState.armConfiguration().setCartImpedance().isCartImpedanceSet())
	        {
	        	
	        	
	        	CartesianImpedanceControlMode cicm = new CartesianImpedanceControlMode();
	        	// These parameters cant be changed online, only stiffnesses and damping, read KUKA documentation. However when we switch Control modes, this values can be set up.
	        	cicm.setMaxCartesianVelocity(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().x(),
										     _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().y(), 
											 _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().z(),
											 _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r3(), 
											 _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r2(), 
											 _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r1());
		        	cicm.setMaxPathDeviation(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().x(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().y(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().z(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r3(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r2(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r1());
	        	cicm.setNullSpaceDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().nullspaceDamping());
	        	cicm.setNullSpaceStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().nullspaceStiffness());
	        	cicm.setMaxControlForce(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().x(), 
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().y(), 
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().z(), 
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r3(), 
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r2(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r1(), true);
	        	
	        	
		         cicm.parametrize(CartDOF.X).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().x());
	             cicm.parametrize(CartDOF.Y).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().y());
	             cicm.parametrize(CartDOF.Z).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().z());
	             cicm.parametrize(CartDOF.A).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r3());
	             cicm.parametrize(CartDOF.B).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r2());
	             cicm.parametrize(CartDOF.C).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r1());
	
		         cicm.parametrize(CartDOF.X).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().x());
	             cicm.parametrize(CartDOF.Y).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().y());
	             cicm.parametrize(CartDOF.Z).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().z());
	             cicm.parametrize(CartDOF.A).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r3());
	             cicm.parametrize(CartDOF.B).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r2());
	             cicm.parametrize(CartDOF.C).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r1());
	              
	             mcm = cicm; 
	             
	             getLogger().info("What is the max control force:" + cicm.getMaxControlForce().toString());
	             getLogger().info("Nullspace stiffness:" + cicm.getNullSpaceStiffness().toString());
	             getLogger().info("Nullspace damping:" + cicm.getNullSpaceDamping().toString());
	        }
	        else
	        {
		        CartesianImpedanceControlMode cicm_def = new CartesianImpedanceControlMode();
		        cicm_def.setMaxCartesianVelocity(1000, 1000, 1000, 6.3, 6.3, 6.3);
		        cicm_def.setMaxPathDeviation(1000., 1000., 1000., 500., 500., 500.);
		        cicm_def.setNullSpaceDamping(0.7);
		        cicm_def.setNullSpaceStiffness(100);
		        cicm_def.setMaxControlForce(200, 200, 200, 200, 200, 200, true);
	        	
		        cicm_def.parametrize(CartDOF.X).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessX());
		        cicm_def.parametrize(CartDOF.Y).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessY());
		        cicm_def.parametrize(CartDOF.Z).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessZ());
		        cicm_def.parametrize(CartDOF.A).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessA());
		        cicm_def.parametrize(CartDOF.B).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessB());
		        cicm_def.parametrize(CartDOF.C).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessC());
		
		        cicm_def.parametrize(CartDOF.X).setDamping(_processDataManager.get_CartesianImpedenceDampingX());
		        cicm_def.parametrize(CartDOF.Y).setDamping(_processDataManager.get_CartesianImpedenceDampingY());
		        cicm_def.parametrize(CartDOF.Z).setDamping(_processDataManager.get_CartesianImpedenceDampingZ());
		        cicm_def.parametrize(CartDOF.A).setDamping(_processDataManager.get_CartesianImpedenceDampingA());
		        cicm_def.parametrize(CartDOF.B).setDamping(_processDataManager.get_CartesianImpedenceDampingB());
		        cicm_def.parametrize(CartDOF.C).setDamping(_processDataManager.get_CartesianImpedenceDampingC());
		          
		          mcm = cicm_def; 
		}
	    	          
		} else if(controlMode==grl.flatbuffer.EControlMode.JOINT_IMP_CONTROL_MODE){
			
			JointImpedanceControlMode jicm =  new JointImpedanceControlMode(_lbr.getJointCount())
									.setStiffnessForAllJoints(_processDataManager.get_JointImpedenceStiffness())
									.setDampingForAllJoints(_processDataManager.get_JointImpedenceDamping());
			// TODO: read relevant stiffness/damping params
			//cicm.parametrize(CartDOF.X).setStiffness(stiffnessX);
			//cicm.parametrize(CartDOF.Y).setStiffness(stiffnessY);
			//cicm.parametrize(CartDOF.Z).setStiffness(stiffnessZ);

			mcm = jicm;
		} 
		else if(controlMode==grl.flatbuffer.EControlMode.CART_FORCE_CONTROL_MODE){
			getLogger().info("In Force Control Mode!!"); 
			if(_currentKUKAiiwaState.armConfiguration().setConstCartForce().isConstFTSet()){
				CartesianSineImpedanceControlMode cscm = new CartesianSineImpedanceControlMode();
//				cscm.setMaxCartesianVelocity(1000, 1000, 1000, 6.3, 6.3, 6.3);
//				cscm.setMaxPathDeviation(1000., 1000., 1000., 500., 500., 500.);
//				cscm.setNullSpaceDamping(0.5);
//				cscm.setNullSpaceStiffness(100);
//				cscm.setMaxControlForce(200, 200, 200, 200, 200, 200, true);
//	        	
//				cscm.parametrize(CartDOF.X).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessX());
//				cscm.parametrize(CartDOF.Y).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessY());
//				cscm.parametrize(CartDOF.Z).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessZ());
//				cscm.parametrize(CartDOF.A).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessA());
//		        cscm.parametrize(CartDOF.B).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessB());
//		        cscm.parametrize(CartDOF.C).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessC());
//		
//		        cscm.parametrize(CartDOF.X).setDamping(_processDataManager.get_CartesianImpedenceDampingX());
//		        cscm.parametrize(CartDOF.Y).setDamping(_processDataManager.get_CartesianImpedenceDampingY());
//		        cscm.parametrize(CartDOF.Z).setDamping(_processDataManager.get_CartesianImpedenceDampingZ());
//		        cscm.parametrize(CartDOF.A).setDamping(_processDataManager.get_CartesianImpedenceDampingA());
//		        cscm.parametrize(CartDOF.B).setDamping(_processDataManager.get_CartesianImpedenceDampingB());
//		        cscm.parametrize(CartDOF.C).setDamping(_processDataManager.get_CartesianImpedenceDampingC());
				
				// These parameters cant be changed online, only stiffnesses and damping, read KUKA documentation. However when we switch Control modes, this values can be set up.
				cscm.setMaxCartesianVelocity(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().x(),
										     _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().y(), 
											 _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().z(),
											 _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r3(), 
											 _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r2(), 
											 _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r1());
				cscm.setMaxPathDeviation(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().x(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().y(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().z(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r3(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r2(),
								             _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r1());
				cscm.setNullSpaceDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().nullspaceDamping());
				cscm.setNullSpaceStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().nullspaceStiffness());
				cscm.setMaxControlForce(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().x(), 
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().y(), 
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().z(), 
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r3(), 
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r2(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r1(), true);
	        	
	        	
				cscm.parametrize(CartDOF.X).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().x());
				cscm.parametrize(CartDOF.Y).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().y());
				cscm.parametrize(CartDOF.Z).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().z());
				cscm.parametrize(CartDOF.A).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r3());
				cscm.parametrize(CartDOF.B).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r2());
				cscm.parametrize(CartDOF.C).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r1());
	
				cscm.parametrize(CartDOF.X).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().x());
				cscm.parametrize(CartDOF.Y).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().y());
				cscm.parametrize(CartDOF.Z).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().z());
				cscm.parametrize(CartDOF.A).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r3());
				cscm.parametrize(CartDOF.B).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r2());
				cscm.parametrize(CartDOF.C).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r1());
		        
				
				getLogger().info("Apply Ctrl F/T in: " +  _currentKUKAiiwaState.armConfiguration().setConstCartForce().CartDof());
				
				
				if( _currentKUKAiiwaState.armConfiguration().setConstCartForce().CartDof().equals("X"))
					cscm = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.X, 
						                                                    _currentKUKAiiwaState.armConfiguration().setConstCartForce().force(), 
						                                                    _currentKUKAiiwaState.armConfiguration().setConstCartForce().stiffness());
				else if( _currentKUKAiiwaState.armConfiguration().setConstCartForce().CartDof().equals("Y"))
					cscm = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.Y, 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().force(), 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().stiffness());
				else if( _currentKUKAiiwaState.armConfiguration().setConstCartForce().CartDof().equals("Z"))
					cscm = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.Z, 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().force(), 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().stiffness());
				
				else if( _currentKUKAiiwaState.armConfiguration().setConstCartForce().CartDof().equals("Rx"))
					cscm = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.C, 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().force(), 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().stiffness());
				
				else if( _currentKUKAiiwaState.armConfiguration().setConstCartForce().CartDof().equals("Ry"))
					cscm = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.B, 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().force(), 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().stiffness());
				
				else if( _currentKUKAiiwaState.armConfiguration().setConstCartForce().CartDof().equals("Rz"))
					cscm = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.A, 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().force(), 
                            _currentKUKAiiwaState.armConfiguration().setConstCartForce().stiffness());
				else
					getLogger().error("Invalid Ctrl F/T DOF set. Please set either X,Y,Z,A,B,C \n ");
				
				mcm = cscm;
			}
			else{
				
				CartesianSineImpedanceControlMode cscm = new CartesianSineImpedanceControlMode();
				
				cscm.setMaxCartesianVelocity(1000, 1000, 1000, 6.3, 6.3, 6.3);
				cscm.setMaxPathDeviation(1000., 1000., 1000., 500., 500., 500.);
				cscm.setNullSpaceDamping(0.5);
				cscm.setNullSpaceStiffness(100);
				cscm.setMaxControlForce(200, 200, 200, 200, 200, 200, true);
	        	
				cscm.parametrize(CartDOF.X).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessX());
				cscm.parametrize(CartDOF.Y).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessY());
				cscm.parametrize(CartDOF.Z).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessZ());
				cscm.parametrize(CartDOF.A).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessA());
		        cscm.parametrize(CartDOF.B).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessB());
		        cscm.parametrize(CartDOF.C).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessC());
		
		        cscm.parametrize(CartDOF.X).setDamping(_processDataManager.get_CartesianImpedenceDampingX());
		        cscm.parametrize(CartDOF.Y).setDamping(_processDataManager.get_CartesianImpedenceDampingY());
		        cscm.parametrize(CartDOF.Z).setDamping(_processDataManager.get_CartesianImpedenceDampingZ());
		        cscm.parametrize(CartDOF.A).setDamping(_processDataManager.get_CartesianImpedenceDampingA());
		        cscm.parametrize(CartDOF.B).setDamping(_processDataManager.get_CartesianImpedenceDampingB());
		        cscm.parametrize(CartDOF.C).setDamping(_processDataManager.get_CartesianImpedenceDampingC());
		        
//				cscm = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.Z, 
//						                                                    _currentKUKAiiwaState.armConfiguration().setConstCartForce().force(), 
//						                                                    _currentKUKAiiwaState.armConfiguration().setConstCartForce().stiffness());
				cscm = CartesianSineImpedanceControlMode.createDesiredForce(CartDOF.Z, 
	                    5, 
	                    1000);
				mcm = cscm;
			}
		}

		return mcm;
	}
	boolean updateConfig(grl.flatbuffer.KUKAiiwaArmConfiguration newConfig){

		if(newConfig == null) return false;

		if(_currentKUKAiiwaState.armConfiguration().controlMode()!=_previousKUKAiiwaState.armConfiguration().controlMode())
		{

			_smartServoRuntime.changeControlModeSettings(_activeMotionControlMode);
		}



		return true;
	}

	/**
	 * main.
	 * 
	 * @param args
	 *            args
	 */
	public static void main(final String[] args)
	{
		final GRL_Driver_ChangeModes app = new GRL_Driver_ChangeModes();
		app.runApplication();
	}

}
