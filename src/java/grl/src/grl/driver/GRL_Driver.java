package grl.driver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import grl.ProcessDataManager;
import grl.StartStopSwitchUI;
import grl.TeachMode;
import grl.UpdateConfiguration;
import grl.flatbuffer.ArmState;
import grl.flatbuffer.CartesianImpedenceControlMode;
import grl.flatbuffer.KUKAiiwaInterface;
import grl.flatbuffer.MoveArmJointServo;
import grl.flatbuffer.MoveArmTrajectory;

import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.zeromq.ZMQ;

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
    // create an JointPosition Instance, to play with
    private JointPosition              _smartServoDestination = null;
    private JointPosition              _jointZeros = new JointPosition(0,0,0,0,0,0,0);
	
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

		// TODO: fix these, right now they're useless
		_flangeAttachment = getApplicationData().createFromTemplate("FlangeAttachment");
		//_updateConfiguration = new UpdateConfiguration(_lbr,_flangeAttachment);
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


		_teachControlMode = new JointImpedanceControlMode(7)
								.setStiffnessForAllJoints(0.1)
								.setDampingForAllJoints(0.7);
		
	}

	@Override
	public void run()
	{

		ZMQ.Context context = ZMQ.context(1);

		getLogger().info("GRL_Driver from github.com/ahundt/grl starting...\nZMQ Connecting to: " + _processDataManager.get_ZMQ_MASTER_URI());
		ZMQ.Socket subscriber = context.socket(ZMQ.DEALER);
		subscriber.connect(_processDataManager.get_ZMQ_MASTER_URI());
		subscriber.setRcvHWM(1000000);

		int statesLength = 0;
		grl.flatbuffer.KUKAiiwaStates currentKUKAiiwaStates = null;
		byte [] data = null;
		ByteBuffer bb = null;

		getLogger().info("Waiting for initialization...");
		while(statesLength<1 && currentKUKAiiwaStates == null){
			if((data = subscriber.recv(ZMQ.DONTWAIT))!=null){
				bb = ByteBuffer.wrap(data);
				getLogger().info("Flatbuffer received");

				currentKUKAiiwaStates = grl.flatbuffer.KUKAiiwaStates.getRootAsKUKAiiwaStates(bb);
				statesLength = currentKUKAiiwaStates.statesLength();
			}

			if (_startStopUI.is_stopped()) {
				getLogger().info("Stopping program.");
				return;
			}
		}

		getLogger().info("States initialized...");

		// create an JointPosition Instance, to play with
		JointPosition destination = new JointPosition(
				_lbr.getJointCount());


		IMotionContainer currentMotion = null;

		boolean stop = false;
		boolean newConfig = false;
		boolean switchingMode = true;

		// TODO: Let user set mode (teach/joint control from tablet as a backup!)
		//this.getApplicationData().getProcessData("DefaultMode").
		
		
		int message_counter = 0;
		// TODO: add a message that we send to the driver with data log strings
		while (!stop && !_startStopUI.is_stopped()) {

			// TODO: IMPORTANT: this recv call must be made asynchronous
			boolean isRecoveryRequired = _pausedApplicationRecovery.isRecoveryRequired();

			// TODO: Allow updates via zmq and tablet
			if((data = subscriber.recv(ZMQ.DONTWAIT))!=null){
				message_counter+=1;
				bb = ByteBuffer.wrap(data);

				currentKUKAiiwaStates = grl.flatbuffer.KUKAiiwaStates.getRootAsKUKAiiwaStates(bb, currentKUKAiiwaStates);

				// TODO: this loop needs to be initialized in the right order
				// and account for runtime changes on tablet and ZMQ, then sync them
				if(currentKUKAiiwaStates.statesLength()>0) {
					// initialize the fist state
					grl.flatbuffer.KUKAiiwaState tmp = currentKUKAiiwaStates.states(0);
					if (tmp == null || tmp.armControlState() == null) {
						if (message_counter % 100 == 0) {
							getLogger().warn("NULL ArmControlState message!");
						}
						continue;
					} else {
						_previousKUKAiiwaState = _currentKUKAiiwaState;
						_currentKUKAiiwaState = tmp;
					}

					synchronized(_lbr) {
						
						if (_currentKUKAiiwaState == null) {
							getLogger().error("Missing current state message!");
							continue;
						}

						// Print out a notice when switching modes
						if( message_counter == 1 || (_previousKUKAiiwaState != null && _previousKUKAiiwaState.armControlState() != null &&
								_currentKUKAiiwaState.armControlState().stateType() != _previousKUKAiiwaState.armControlState().stateType()))
						{
							getLogger()
							.info("Switching mode: "
									+ ArmState.name(_currentKUKAiiwaState.armControlState().stateType()));
							switchingMode = true;
						} else {
							switchingMode = false;
						}

						if(_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.ShutdownArm){
							///////////////////////////////////////////////
							// ShutdownArm
							getLogger()
							.info("ShutdownArm command received, stopping GRL_Driver...");
							stop = true;
						}
						else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.MoveArmTrajectory) {
							///////////////////////////////////////////////
							// MoveArmTrajectory mode (sequence of joint angles)
							
							// TODO: not fully implemented
							_smartServoRuntime.stopMotion();
							if (currentMotion != null) {
								currentMotion.cancel();
							}


							MoveArmTrajectory mat;
							if(_currentKUKAiiwaState.armControlState() != null) {
							 mat = (MoveArmTrajectory)_currentKUKAiiwaState.armControlState().state(new MoveArmTrajectory());
							} else {
								getLogger().error("Received null armControlState in servo!");
								continue;
							
							}

							for (int j = 0; j < mat.trajLength(); j++) {

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
							// MoveArmJointServo mode
							
							if (currentMotion != null) {
								currentMotion.cancel();
							}

							MoveArmJointServo mas;
							if(_currentKUKAiiwaState.armControlState() != null) {
							 mas = (MoveArmJointServo)_currentKUKAiiwaState.armControlState().state(new MoveArmJointServo());
							} else {
								getLogger().error("Received null armControlState in servo!");
								continue;
							
							}
							
							// start up the motion if not enabled yet
							if( _smartServoMotion == null) {
								// make sure this is up
								// also make sure this is running
						        destination = new JointPosition(
						                _lbr.getCurrentJointPosition());
						        
						        _smartServoMotion = new SmartServo(destination);
						        _smartServoMotionControlMode = new JointImpedanceControlMode(7)
														.setStiffnessForAllJoints(50)
														.setDampingForAllJoints(0.7);

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
						        
						        // Set the motion properties to 20% of systems abilities
						        // TODO: load these over C++ interface
						        _smartServoMotion
						        	.setJointAccelerationRel(0.2)
						        	.setJointVelocityRel(0.2)
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
								}
							}
							
							grl.flatbuffer.JointState jointState = mas.goal();
							if(jointState.positionLength()!=destination.getAxisCount()){
								getLogger().error("Didn't receive correct number of joints! skipping to start of loop...");
								continue;
							}
							//String pos = "pos:";
							for (int k = 0; k < destination.getAxisCount(); ++k)
							{
								double position = jointState.position(k);
								destination.set(k, position);
								//pos = " " + k + ": " + position;
							}
							
							if(_lbrInterface==grl.flatbuffer.KUKAiiwaInterface.FRI){

									FRISession friSession = _updateConfiguration.get_FRISession();
									FRIJointOverlay motionOverlay = new FRIJointOverlay(friSession);

									try {
										friSession.await(10, TimeUnit.SECONDS);

										currentMotion = _lbr.moveAsync(positionHold(_activeMotionControlMode, -1, TimeUnit.SECONDS).addMotionOverlay(motionOverlay));
									} catch (TimeoutException e) {
										e.printStackTrace();
										friSession.close();
										return;
									}
							} else {
								// TODO: we need to make sure this is running, and we need to cancel the current motion
								try {
									_smartServoRuntime.setDestination(destination);
								} catch (java.lang.IllegalStateException ex) {
									getLogger().error("Could not update smart servo destination! Clearing SmartServo.");
									_smartServoMotion = null;
									_smartServoRuntime = null;
									getLogger().error(ex.getMessage());
								}
						   }

						} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.StopArm) {

							_smartServoRuntime.stopMotion();
							if (currentMotion != null) {
								currentMotion.cancel();
							}
							//tm.setActive(false);

						} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.TeachArm) {
							///////////////////////////////////////////////
							// Teach mode

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
									getLogger().info("Destroying smart servo!");
								}

								getLogger().warn("Enabling Teach Mode (grav comp): current = " +
										_currentKUKAiiwaState.armControlState().stateType());

								// trying to use kuka's provided handguidingmotion but it isn't working now.
								// using an if statement to default to old behavior.
								boolean useHandGuidingMotion = true;
								
								if(useHandGuidingMotion)
								{
									// see kuka documentation 1.9 for details
									_handGuidingMotion = handGuiding()
									   .setAxisLimitsMax(_maxAllowedJointLimits)
									   .setAxisLimitsMin(_minAllowedJointLimits)
									.setAxisLimitsEnabled(true, true, true, true, true, true, true)
									.setAxisLimitViolationFreezesAll(false).setPermanentPullOnViolationAtStart(true);
									currentMotion = _toolAttachedToLBR.moveAsync(_handGuidingMotion);//move(_handGuidingMotion);
									//getLogger().info("Done hand guiding");
								}
								else
								{
									currentMotion = _toolAttachedToLBR.moveAsync(positionHold(_teachControlMode, -1, TimeUnit.SECONDS));
								}
							}
							
							//_toolAttachedToLBR.
						} else {
							System.out.println("Unsupported Mode! stopping");
							stop = true;
						}
					}
				}
			}
		} // end primary while loop


		// done
		subscriber.close();
		context.term();
		if (_updateConfiguration!=null && _updateConfiguration.get_FRISession() != null) {
			_updateConfiguration.get_FRISession().close();
		}
		
		getLogger()
		.info("ZMQ connection closed.\nExiting...\nThanks for using the\nGRL_Driver from github.com/ahundt/grl\nGoodbye!");
		//System.exit(1);
	}


	boolean updateConfig(grl.flatbuffer.KUKAiiwaArmConfiguration newConfig){

		if(newConfig == null) return false;

		if(_currentKUKAiiwaState.armConfiguration().controlMode()!=_previousKUKAiiwaState.armConfiguration().controlMode())
		{
			if(newConfig.controlMode()==grl.flatbuffer.EControlMode.POSITION_CONTROL_MODE){
				_activeMotionControlMode = new PositionControlMode();
			} else if(newConfig.controlMode()==grl.flatbuffer.EControlMode.CART_IMP_CONTROL_MODE){
				CartesianImpedanceControlMode cicm = new CartesianImpedanceControlMode();
				// TODO: read relevant stiffness/damping params
				//cicm.parametrize(CartDOF.X).setStiffness(stiffnessX);
				//cicm.parametrize(CartDOF.Y).setStiffness(stiffnessY);
				//cicm.parametrize(CartDOF.Z).setStiffness(stiffnessZ);

				// TODO: make motion control mode configurable over zmq interface
				_smartServoMotionControlMode = new CartesianImpedanceControlMode()
											.setMaxCartesianVelocity(500, 500, 500, 1, 1, 1)
											.setMaxPathDeviation(1000, 1000, 1000, 5, 5, 5)
											.setNullSpaceDamping(0.5)
											.setNullSpaceStiffness(2)
											.setMaxControlForce(50, 50, 50, 100, 100, 100, true);

				_activeMotionControlMode = cicm;
			} else if(newConfig.controlMode()==grl.flatbuffer.EControlMode.JOINT_IMP_CONTROL_MODE){
				JointImpedanceControlMode cicm = new JointImpedanceControlMode();
				// TODO: read relevant stiffness/damping params
				//cicm.parametrize(CartDOF.X).setStiffness(stiffnessX);
				//cicm.parametrize(CartDOF.Y).setStiffness(stiffnessY);
				//cicm.parametrize(CartDOF.Z).setStiffness(stiffnessZ);

				_activeMotionControlMode = cicm;
			}

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
