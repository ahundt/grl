package grl.driver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

//uncomment for sunrise OS 1.9 and below
//import static com.kuka.roboticsAPI.motionModel.MMCMotions.handGuiding;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

import grl.ProcessDataManager;
import grl.StartStopSwitchUI;
import grl.TeachMode;
import grl.UpdateConfiguration;
import grl.flatbuffer.ArmState;
import grl.flatbuffer.MoveArmJointServo;

import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.zeromq.ZMQ;

import com.google.flatbuffers.Table;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.recovery.IRecovery;
import com.kuka.roboticsAPI.deviceModel.JointLimits;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * Creates a FRI Session.
 */
public class GRL_HandGuiding extends RoboticsAPIApplication
{
	private ProcessDataManager _processDataManager = null; // Stores variables that can be modified by teach pendant in "Process Data" Menu
	private Controller _lbrController;
	private LBR _lbr;
	private StartStopSwitchUI _startStopUI = null;
	private ISmartServoRuntime theSmartServoRuntime = null;
	private AbstractMotionControlMode _activeMotionControlMode;
	private UpdateConfiguration _updateConfiguration;
	private IRecovery _pausedApplicationRecovery = null;
	private PhysicalObject _toolAttachedToLBR;
	private HandGuidingMotion _handGuidingMotion;
	/**
	 *  gripper or other physically attached object
	 *  see "Template Data" panel in top right pane
	 *  of Sunrise Workbench. This can't be created
	 *  at runtime so we create one for you.
	 */
	private Tool    _flangeAttachment;
	private JointLimits _jointLimits;
	private double[] _maxAllowedJointLimits;
	private double[] _minAllowedJointLimits;

	@Override
	public void initialize()
	{
		_startStopUI = new StartStopSwitchUI(this);
		_processDataManager = new ProcessDataManager(this);
		_lbrController = (Controller) getContext().getControllers().toArray()[0];
		_lbr = (LBR) _lbrController.getDevices().toArray()[0];

		// TODO: fix these, right now they're useless
		_flangeAttachment = getApplicationData().createFromTemplate("FlangeAttachment");
		//_updateConfiguration = new UpdateConfiguration(_lbr,_flangeAttachment);
		_pausedApplicationRecovery = getRecovery();

		LoadData _loadData = new LoadData();
		_loadData.setMass(_processDataManager.getEndEffectorWeight());
		_loadData.setCenterOfMass(_processDataManager.getEndEffectorX(),
				_processDataManager.getEndEffectorY(),
				_processDataManager.getEndEffectorZ());
		_toolAttachedToLBR = new Tool("Tool", _loadData);
		_toolAttachedToLBR.attachTo(_lbr.getFlange());
		


		_jointLimits = _lbr.getJointLimits();
		
		
		// used when setting limits in _HandGuidingMotion
		_maxAllowedJointLimits = _jointLimits.getMaxJointPosition().get();
		_minAllowedJointLimits = _jointLimits.getMinJointPosition().get();
		for (int i = 0; i < _lbr.getJointCount(); i++) {
			_maxAllowedJointLimits[i] -= 0.1;
			_minAllowedJointLimits[i] += 0.1;
		}


	}

	@Override
	public void run()
	{

		int statesLength = 0;
		byte [] data = null;
		ByteBuffer bb = null;

		getLogger().info("States initialized...");
		JointPosition destination = new JointPosition(
				_lbr.getJointCount());


		IMotionContainer currentMotion = null;

		boolean stop = false;
		boolean newConfig = false;

		// TODO: Let user set mode (teach/joint control from tablet as a backup!)
		//this.getApplicationData().getProcessData("DefaultMode").

		while(!stop){
			getLogger().warn("Enabling Teach Mode (grav comp)");

			JointImpedanceControlMode controlMode2 = new JointImpedanceControlMode(7); // TODO!!
			controlMode2.setStiffnessForAllJoints(0.1);
			controlMode2.setDampingForAllJoints(0.7);
			//_lbr.moveAsync(positionHold(controlMode2, -1, TimeUnit.SECONDS));

			//HandGuidingMotion handGuidingMotion = new HandGuidingMotion();
			//_toolAttachedToLBR.move(handGuidingMotion);
			

			// see kuka documentation 1.9 for details
			_handGuidingMotion = handGuiding()
			   .setAxisLimitsMax(_maxAllowedJointLimits)
			   .setAxisLimitsMin(_minAllowedJointLimits)
			.setAxisLimitsEnabled(true, true, true, true, true, true, true)
			.setAxisLimitViolationFreezesAll(false).setPermanentPullOnViolationAtStart(true);
			currentMotion = _toolAttachedToLBR.move(_handGuidingMotion);//move(_handGuidingMotion);
			getLogger().info("Done hand guiding");
			
		}

	}



	/**
	 * main.
	 * 
	 * @param args
	 *            args
	 */
	public static void main(final String[] args)
	{
		final GRL_HandGuiding app = new GRL_HandGuiding();
		app.runApplication();
	}

}
