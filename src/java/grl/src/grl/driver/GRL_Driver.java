package grl.driver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import grl.UpdateConfiguration;
import grl.flatbuffer.MoveArmJointServo;

import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.zeromq.ZMQ;

import com.google.flatbuffers.Table;
import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRIJointOverlay;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.recovery.IRecovery;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;


/**
 * Creates a FRI Session.
 */
public class GRL_Driver extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String __controllingLaptopIPAddress;
    private String _RobotIPAddress;
    private String _FRI_KONI_RobotIPAddress;
    private String _FRI_KONI_LaptopIPAddress;
    private ISmartServoRuntime theSmartServoRuntime = null;
	private String _controllingLaptopIPAddressConfig;
	private grl.flatbuffer.KUKAiiwaArmConfiguration _currentActiveConfiguration = null;
	private AbstractMotionControlMode _activeMotionControlMode;
	private UpdateConfiguration _updateConfiguration;
	private IRecovery _pausedApplicationRecovery = null;

	/**
	 *  gripper or other physically attached object
	 *  see "Template Data" panel in top right pane
	 *  of Sunrise Workbench. This can't be created
	 *  at runtime so we create one for you.
	 */
	private Tool    _flangeAttachment;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        _flangeAttachment = getApplicationData().createFromTemplate("FlangeAttachment");


        _updateConfiguration = new UpdateConfiguration(_lbr,_flangeAttachment);
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        __controllingLaptopIPAddress = getApplicationData().getProcessData("Laptop_IP").getValue(); //"192.170.10.100";

        
        // **********************************************************************
        // *** change next line to the KUKA address and Port Number           ***
        // **********************************************************************
        _RobotIPAddress = getApplicationData().getProcessData("Robot_IP").getValue(); //"tcp://172.31.1.100:30010";

        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _FRI_KONI_LaptopIPAddress = getApplicationData().getProcessData("Laptop_KONI_FRI_IP").getValue(); //"192.170.10.100";

        
        // **********************************************************************
        // *** change next line to the KUKA address and Port Number           ***
        // **********************************************************************
        _FRI_KONI_RobotIPAddress = getApplicationData().getProcessData("Robot_KONI_FRI_IP").getValue(); //"tcp://172.31.1.100:30010";
        
        
        _pausedApplicationRecovery = getRecovery();
    }

    @Override
    public void run()
    {

        ZMQ.Context context = ZMQ.context(1);
        

        ZMQ.Socket subscriber = context.socket(ZMQ.DEALER);
        subscriber.connect(_RobotIPAddress);
        subscriber.setRcvHWM(1000);
        
        int statesLength = 0;
        grl.flatbuffer.KUKAiiwaStates currentKUKAiiwaStates = null;
        byte [] data = null;
        ByteBuffer bb = null;
        
        while(statesLength<1 && currentKUKAiiwaStates == null){
            data = subscriber.recv();
            bb = ByteBuffer.wrap(data);

            currentKUKAiiwaStates = grl.flatbuffer.KUKAiiwaStates.getRootAsKUKAiiwaStates(bb);
            statesLength = currentKUKAiiwaStates.statesLength();
        }
        // TODO: support more than one state per message?
        grl.flatbuffer.KUKAiiwaState currentKUKAiiwaState = currentKUKAiiwaStates.states(0);
        
        // TODO: remove default start pose
        // move do default start pose
        //_toolAttachedToLBR.move(ptp(Math.toRadians(10), Math.toRadians(10), Math.toRadians(10), Math.toRadians(-90), Math.toRadians(10), Math.toRadians(10),Math.toRadians(10)));

        
        
       // Prepare ZeroMQ context and dealer
        //getArmConfiguration(configSubscriber);

        JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());
        SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 20% of systems abilities
        double jointVelRel = getApplicationData().getProcessData("jointVelRel").getValue();
        double jointAccRel = getApplicationData().getProcessData("jointAccRel").getValue();
        aSmartServoMotion.setJointAccelerationRel(jointVelRel);
        aSmartServoMotion.setJointVelocityRel(jointAccRel);

        // TODO: read from SmartServo config
        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

        //_toolAttachedToLBR.getDefaultMotionFrame().moveAsync(aSmartServoMotion);
        
        // Fetch the Runtime of the Motion part
        theSmartServoRuntime = aSmartServoMotion.getRuntime();
        
     	// create an JointPosition Instance, to play with
        JointPosition destination = new JointPosition(
                _lbr.getJointCount());
        
        
        TeachMode tm = new TeachMode(_lbr);
        Thread teachModeThread = new Thread(tm);
        teachModeThread.start();
        

        IMotionContainer currentMotion = null;
        
        boolean stop = false;
        boolean newIncomingKUKAiiwaState = false;
        boolean newConfig = false;
        
        grl.flatbuffer.ArmState state = null;
        grl.flatbuffer.ArmControlState armControlState = null;
        grl.flatbuffer.ArmState        armState = null;
        
        // Receive Flat Buffer and Move to Position
        // TODO: add a message that we send to the driver with data log strings
        while (!stop) {
        	// TODO: IMPORTANT: this recv call must be made asynchronous
        	boolean isRecoveryRequired = _pausedApplicationRecovery.isRecoveryRequired();
        	
            if((data = subscriber.recv(ZMQ.DONTWAIT))!=null){
            	newIncomingKUKAiiwaState = true;
                bb = ByteBuffer.wrap(data);
            
                currentKUKAiiwaState.armControlState(armControlState);
            
	            synchronized(_lbr) {
		            bb = ByteBuffer.wrap(data);
		
		            currentKUKAiiwaStates = grl.flatbuffer.KUKAiiwaStates.getRootAsKUKAiiwaStates(bb, currentKUKAiiwaStates);
		            
		            if(currentKUKAiiwaStates.statesLength()>0)

		                // TODO: support more than one state per message?
		                currentKUKAiiwaState = currentKUKAiiwaStates.states(0);
		                currentKUKAiiwaState.armControlState(armControlState);
		            
		            if(armControlState.stateType() == grl.flatbuffer.ArmState.ShutdownArm){
		            	stop = true;
		            }
		            else if (armControlState.stateType() == grl.flatbuffer.ArmState.MoveArmTrajectory) {
		            	tm.setActive(false);
		            	theSmartServoRuntime.stopMotion();
		            	if (currentMotion != null) {
		            		currentMotion.cancel();
		            	}
		            	
		            	grl.flatbuffer.MoveArmTrajectory maj = null;
		            	armControlState.state(maj);
		
			            for (int j = 0; j < maj.trajLength(); j++) {
			            	
			            	JointPosition pos = new JointPosition(_lbr.getCurrentJointPosition());
			            	
				            for (int k = 0; k < destination.getAxisCount(); ++k)
				            {
				            	//destination.set(k, maj.traj(j).position(k));
				            	pos.set(k, maj.traj(j).position(k));
					        	currentMotion = _lbr.moveAsync(ptp(pos));
				            }
				            
			            }
		            } else if (armControlState.stateType() == grl.flatbuffer.ArmState.MoveArmJointServo) {
		            	if (currentMotion != null) {
		            		currentMotion.cancel();
		            	}
		            	tm.setActive(false);
		            	
		            	
		            	
		            	MoveArmJointServo mas = null;
		            	armControlState.state(mas);
		            	
		            	if(_currentActiveConfiguration.commandInterface() == grl.flatbuffer.KUKAiiwaInterface.SmartServo){
		            		grl.flatbuffer.JointState jointState = mas.goal();
			            	
			            	for (int k = 0; k < destination.getAxisCount(); ++k)
			                {
			                	destination.set(k, jointState.position(k));
			                }
			                theSmartServoRuntime.setDestination(destination);
		            		
		            	} else if(_currentActiveConfiguration.commandInterface()==grl.flatbuffer.KUKAiiwaInterface.FRI){
	
		            		FRISession friSession = _updateConfiguration.get_FRISession();
		            		FRIJointOverlay motionOverlay = new FRIJointOverlay(friSession);
		            		
		            		 try {
		            			friSession.await(10, TimeUnit.SECONDS);
	
			            		_lbr.moveAsync(positionHold(_activeMotionControlMode, -1, TimeUnit.SECONDS).addMotionOverlay(motionOverlay));
		            		} catch (TimeoutException e) {
		            			// TODO Automatisch generierter Erfassungsblock
		            			e.printStackTrace();
		            			friSession.close();
		            			return;
		            		}
		            		 
		            	}
		            	
		            } else if (armControlState.stateType() == grl.flatbuffer.ArmState.StopArm) {
		            	theSmartServoRuntime.stopMotion();
		            	if (currentMotion != null) {
		            		currentMotion.cancel();
		            	}
		            	tm.setActive(false);
		            	
		            } else if (armControlState.stateType() == grl.flatbuffer.ArmState.TeachArm) {
		            	theSmartServoRuntime.stopMotion();
		            	if (currentMotion != null) {
		            		currentMotion.cancel();
		            	}
		            	tm.setActive(true);
		            } else {
		            	System.out.println("Unsupported Mode! stopping");
		            	stop = true;
		            }
		            
		            // done processing new command
		            newIncomingKUKAiiwaState = false;
	            }
        }}
            
        
        // done
        subscriber.close();
        context.term();
        _updateConfiguration.get_FRISession().close();
        System.exit(1);
    }
    
    
    boolean updateConfig(grl.flatbuffer.KUKAiiwaArmConfiguration newConfig){
    	
    	if(newConfig == null) return false;
    	
    	if(_currentActiveConfiguration.controlMode()!=newConfig.controlMode())
    	{
    		if(newConfig.controlMode()==grl.flatbuffer.EControlMode.POSITION_CONTROL_MODE){
    			_activeMotionControlMode = new PositionControlMode();
    		} else if(newConfig.controlMode()==grl.flatbuffer.EControlMode.CART_IMP_CONTROL_MODE){
    			CartesianImpedanceControlMode cicm = new CartesianImpedanceControlMode();
    			// TODO: read relevant stiffness/damping params
    			//cicm.parametrize(CartDOF.X).setStiffness(stiffnessX);
    			//cicm.parametrize(CartDOF.Y).setStiffness(stiffnessY);
    			//cicm.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
    			
    			_activeMotionControlMode = cicm;
    		} else if(newConfig.controlMode()==grl.flatbuffer.EControlMode.JOINT_IMP_CONTROL_MODE){
    			JointImpedanceControlMode cicm = new JointImpedanceControlMode();
    			// TODO: read relevant stiffness/damping params
    			//cicm.parametrize(CartDOF.X).setStiffness(stiffnessX);
    			//cicm.parametrize(CartDOF.Y).setStiffness(stiffnessY);
    			//cicm.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
    			
    			_activeMotionControlMode = cicm;
    		}
    		
    		theSmartServoRuntime.changeControlModeSettings(_activeMotionControlMode);
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
