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
import grl.flatbuffer.MoveArmJointServo;
import grl.flatbuffer.MoveArmTrajectory;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.google.flatbuffers.Table;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.ServoMotionJP;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.recovery.IRecovery;
import com.kuka.roboticsAPI.deviceModel.JointLimits;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
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
import com.kuka.roboticsAPI.motionModel.controlModeModel.HandGuidingControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;

/**
 * Creates a FRI Session.
 */
public class GRL_Driver extends RoboticsAPIApplication
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

	private boolean _canServo = true;
	// when we receive a message
	int message_counter = 0;
	int message_counter_since_last_mode_change = 0;
	
	@Override
	public void initialize()
	{
		_startStopUI = new StartStopSwitchUI(this);
		_processDataManager = new ProcessDataManager(this);
		_lbrController = (Controller) getContext().getControllers().toArray()[0];
		_lbr = (LBR) _lbrController.getDevices().toArray()[0];
		

        _friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _processDataManager.get_FRI_KONI_LaptopIPAddress());
        _friConfiguration.setSendPeriodMilliSec(4);

        _friSession = new FRISession(_friConfiguration);

		_flangeAttachment = getApplicationData().createFromTemplate("FlangeAttachment");
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
		boolean switchingMode = true;

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
				switchingMode = true;
				message_counter_since_last_mode_change  = 0;
			} else {

				message_counter_since_last_mode_change++;
				switchingMode = false;
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
				// Teach mode
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
					
					if (_smartServoRuntime != null) {
						_smartServoRuntime.stopMotion();
						_smartServoMotion = null;
						_smartServoRuntime = null;
						getLogger().info("Ending smart servo!");
					}


					getLogger().warn("Enabling Teach Mode with gravity compensation. mode id code = " +
							_currentKUKAiiwaState.armControlState().stateType());

					// trying to use kuka's provided handguidingmotion but it isn't working now.
					// using an if statement to default to old behavior.
					boolean useHandGuidingMotion = true;
					
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
				
				//_toolAttachedToLBR.
			}
			else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.MoveArmTrajectory) {
				///////////////////////////////////////////////
				// MoveArmTrajectory mode (sequence of joint angles)
				///////////////////////////////////////////////

				// create an JointPosition Instance, to play with
				JointPosition destination = new JointPosition(
						_lbr.getJointCount());
				// TODO: not fully implemented
				_smartServoRuntime.stopMotion();
				if (currentMotion != null) 
				{
					currentMotion.cancel();
				}
				if(!cancelTeachMode()) continue;


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

				// create an JointPosition Instance, to play with
				JointPosition destination = new JointPosition(
						_lbr.getJointCount());
				if (currentMotion != null) {
					currentMotion.cancel();
				}
				if(!cancelTeachMode()) continue;
				
				//if (!_canServo) {
				//	if(message_counter % 1000 == 0) getLogger().error("Cannot servo until teach move completed!");
				//	continue;
				//}

				MoveArmJointServo mas;
				if(_currentKUKAiiwaState.armControlState() != null) {
					mas = (MoveArmJointServo)_currentKUKAiiwaState.armControlState().state(new MoveArmJointServo());
				} else {
					getLogger().error("Received null armControlState in servo!");
					continue;
					//return;
				}
				
				// start up the motion if not enabled yet
				if( _smartServoMotion == null) {
					// make sure this is up
					// also make sure this is running
			        destination = new JointPosition(
			                _lbr.getCurrentJointPosition());
			        
			        _smartServoMotion = new SmartServo(destination);

					// TODO: support more control modes & zmq interface
					_smartServoMotionControlMode = getMotionControlMode(grl.flatbuffer.EControlMode.CART_IMP_CONTROL_MODE);
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
			            else
			            {
			            	_smartServoMotion.setMode(_smartServoMotionControlMode);
			            }
			        }
			        catch (IllegalStateException e)
			        {
			            getLogger().info("Omitting validation failure for this sample\n"
			                    + e.getMessage());
			        }
			        
			        // Set the motion properties to % of system abilities. For example .2 is 20% of systems abilities
			        // TODO: load these over C++ interface
			        _smartServoMotion
			        	.setJointAccelerationRel(_processDataManager.get_jointAccelRel())
			        	.setJointVelocityRel(_processDataManager.get_jointVelRel())
			        	.setMinimumTrajectoryExecutionTime(20e-3);

			        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(_smartServoMotion);
			        getLogger().info("Setting up SmartServo");
			        
				}
				
				if (_smartServoRuntime == null) {
					getLogger().info("Setting up Smart Servo runtime");
					try {
						// TODO: make motion control mode configurable over zmq interface
						_smartServoRuntime = _smartServoMotion.getRuntime();
				        // _smartServoRuntime.changeControlModeSettings(_smartServoMotionControlMode);
				        _activeMotionControlMode = _smartServoMotionControlMode;
					} catch (java.lang.IllegalStateException ex) {
						getLogger().error("Could not retrieve SmartServo runtime!");
						_smartServoRuntime = null;
						_smartServoMotion = null;
						getLogger().error(ex.getMessage());
						continue;
						//return;
					}
				}
				
				grl.flatbuffer.JointState jointState = mas.goal();
				if(jointState.positionLength()!=destination.getAxisCount()){
					if(message_counter % 1000 == 0) getLogger().error("Didn't receive correct number of joints! skipping to start of loop...");
					continue;
					//return;
				}
				String pos = "pos:";
				for (int k = 0; k < destination.getAxisCount(); ++k)
				{
					double position = jointState.position(k);
					destination.set(k, position);
				    pos = pos + " " + k + ": " + position;
				}
				
				if(_lbrInterface==grl.flatbuffer.KUKAiiwaInterface.FRI){

					_friSession = _updateConfiguration.get_FRISession();
					FRIJointOverlay motionOverlay = new FRIJointOverlay(_friSession);

					try {
						_friSession.await(10, TimeUnit.SECONDS);

						currentMotion = _lbr.moveAsync(positionHold(_activeMotionControlMode, -1, TimeUnit.SECONDS).addMotionOverlay(motionOverlay));
					} catch (TimeoutException e) {
						getLogger().error("FRISession timed out, closing...");
						e.printStackTrace();
						_friSession.close();
						return;
					}
				} else if(_smartServoRuntime != null )  {
					try {
						if(message_counter % 1000 == 0) getLogger().info("Setting Smart Servo Joint destination to " + pos);
						_smartServoRuntime.setDestination(destination);
					} catch (java.lang.IllegalStateException ex) {
						getLogger().error("Could not update smart servo destination! Clearing SmartServo.");
						_smartServoMotion = null;
						_smartServoRuntime = null;
						getLogger().error(ex.getMessage());
					}
			   } else {
					getLogger().error("Couldn't issue motion command, smartServo motion was most likely reset. retrying...");
			   }

			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.StopArm) {

				_smartServoRuntime.stopMotion();
				if (currentMotion != null) {
					currentMotion.cancel();
				}
				if(!cancelTeachMode()) continue;
				
				//tm.setActive(false);
			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.PauseArm) {
				if (_smartServoRuntime != null) {
					_smartServoRuntime.stopMotion();
				}
				if (currentMotion != null) {
					currentMotion.cancel();
				}
				if(!cancelTeachMode()) continue;
				
				PositionControlMode controlMode = new PositionControlMode();
				if(message_counter_since_last_mode_change < 2 || message_counter_since_last_mode_change % 100 == 0){
					_lbr.move(positionHold(controlMode,10,TimeUnit.MILLISECONDS));
				}

			} else {
				System.out.println("Unsupported Mode! stopping");
				stop = true;
			}
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

			if(!_teachModeRunnable.isEnableEnded() && 
				_currentKUKAiiwaState.armControlState().stateType() != grl.flatbuffer.ArmState.TeachArm){
				
				if(message_counter % 30 == 0) getLogger().warn("Can't Exit Teach Mode Yet,\n Did You Press The Hand Guiding Button?");
				return false;
			}
		}
		return true;
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

			// TODO: make motion control mode configurable over zmq interface
			/// @note setMaxCartesianVelocity STOPS THE ROBOT ABOVE THAT VELOCITY RATHER THAN CAPPING THE VELOCITY
	        CartesianImpedanceControlMode cicm = new CartesianImpedanceControlMode()
										.setMaxCartesianVelocity(1000, 1000, 1000, 6.3, 6.3, 6.3)
										.setMaxPathDeviation(1000, 1000, 1000, 5, 5, 5)
										.setNullSpaceDamping(0.5)
										.setNullSpaceStiffness(2)
										.setMaxControlForce(200, 200, 200, 200, 200, 200, true);
	        
	        cicm.parametrize(CartDOF.X).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessX());
            cicm.parametrize(CartDOF.Y).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessY());
            cicm.parametrize(CartDOF.Z).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessZ());
            cicm.parametrize(CartDOF.A).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessA());
            cicm.parametrize(CartDOF.B).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessB());
            cicm.parametrize(CartDOF.C).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessC());

	        cicm.parametrize(CartDOF.X).setDamping(_processDataManager.get_CartesianImpedenceDampingX());
            cicm.parametrize(CartDOF.Y).setDamping(_processDataManager.get_CartesianImpedenceDampingY());
            cicm.parametrize(CartDOF.Z).setDamping(_processDataManager.get_CartesianImpedenceDampingZ());
            cicm.parametrize(CartDOF.A).setDamping(_processDataManager.get_CartesianImpedenceDampingA());
            cicm.parametrize(CartDOF.B).setDamping(_processDataManager.get_CartesianImpedenceDampingB());
            cicm.parametrize(CartDOF.C).setDamping(_processDataManager.get_CartesianImpedenceDampingC());

			mcm = cicm;
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
		final GRL_Driver app = new GRL_Driver();
		app.runApplication();
	}

}
