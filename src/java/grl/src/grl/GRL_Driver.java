package grl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

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
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

import grl.zmqDriver.TeachMode;
import grl.zmqDriver.UpdateConfigurationRunnable;

/**
 * Creates a FRI Session.
 */
public class GRL_Driver extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _hostName;
    private String _controllingLaptopIPAddress;
    private PhysicalObject _toolAttachedToLBR;
    private ISmartServoRuntime theSmartServoRuntime = null;
	private String _controllingLaptopIPAddressConfig;
	private grl.flatbuffer.KUKAiiwaArmConfiguration _currentActiveConfiguration = null;
	private AbstractMotionControlMode _activeMotionControlMode;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _hostName = getApplicationData().getProcessData("hostAddress").getValue(); //"192.170.10.100";
        
        // **********************************************************************
        // *** change next line to the KUKA address and Port Number           ***
        // **********************************************************************
        _controllingLaptopIPAddress = getApplicationData().getProcessData("controlAddress").getValue(); //"tcp://172.31.1.100:30010";
        _controllingLaptopIPAddressConfig = getApplicationData().getProcessData("configAddress").getValue(); //"tcp://172.31.1.100:30011";
        

        // FIXME: Set proper Weights or use the plugin feature
        double[] translationOfTool =
        { 0, 0, 0 };
        double mass = 0;
        double[] centerOfMassInMillimeter =
        { 0, 0, 0 };
        _toolAttachedToLBR = ServoMotionUtilities.createTool(_lbr,
                "SimpleJointMotionSampleTool", translationOfTool, mass,
                centerOfMassInMillimeter);
        
    }

    @Override
    public void run()
    {

        ZMQ.Context context = ZMQ.context(1);
        
        ZMQ.Socket configSubscriber = context.socket(ZMQ.DEALER);
        configSubscriber.connect(_controllingLaptopIPAddressConfig);
        configSubscriber.setRcvHWM(1000000);

        UpdateConfigurationRunnable ucr = new UpdateConfigurationRunnable(configSubscriber,_lbr,_toolAttachedToLBR);
        // blocking call, don't really run until at least one configuration message was received
        ucr.getOneUpdate();
        grl.flatbuffer.KUKAiiwaArmConfiguration latestConfig = ucr.getLatestConfiguration();
        Thread updateConfigThread = new Thread(ucr);
        updateConfigThread.start();
        

        ZMQ.Socket subscriber = context.socket(ZMQ.DEALER);
        subscriber.connect(_controllingLaptopIPAddress);
        subscriber.setRcvHWM(1000000);
        

    	// TODO: IMPORTANT: this recv call must be made asynchronous
        byte [] data = subscriber.recv();
        ByteBuffer bb = ByteBuffer.wrap(data);
        
        // TODO: remove default start pose
        // move do default start pose
        //_toolAttachedToLBR.move(ptp(Math.toRadians(10), Math.toRadians(10), Math.toRadians(10), Math.toRadians(-90), Math.toRadians(10), Math.toRadians(10),Math.toRadians(10)));

        
        // Configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _hostName);
        grl.flatbuffer.FRI friConfigBuf = null;
        friConfigBuf = latestConfig.FRIConfig(friConfigBuf);
        if(friConfigBuf !=null && friConfigBuf.sendPeriodMillisec() >0){
            friConfiguration.setSendPeriodMilliSec(friConfigBuf.sendPeriodMillisec());
        }
        
        FRISession friSession = new FRISession(friConfiguration);
        
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
        

        grl.flatbuffer.ArmControlState armControlState = grl.flatbuffer.ArmControlState.getRootAsArmControlState(bb);
        IMotionContainer currentMotion = null;
        
        
        // TODO: Current active configuration and associated real application states should now be fully set before main loop.
        _currentActiveConfiguration = latestConfig;
        
        boolean stop = false;
        boolean newCommand = false;
        boolean newConfig = false;
        
        // Receive Flat Buffer and Move to Position
        // TODO: add a message that we send to the driver with data log strings
        while (!stop) {
        	// TODO: IMPORTANT: this recv call must be made asynchronous
        	
        	if(ucr.haveNewConfiguration()) {
        		latestConfig = ucr.getLatestConfiguration();
        		newConfig = true;
        	}
        	
            if((data = subscriber.recv(ZMQ.DONTWAIT))!=null){
            	newCommand = true;
                bb = ByteBuffer.wrap(data);
            
            
	            synchronized(_lbr) {
		            bb = ByteBuffer.wrap(data);
		
		            armControlState = grl.flatbuffer.ArmControlState.getRootAsArmControlState(bb);
		            long num = armControlState.sequenceNumber();
		            double time = armControlState.timeStamp();
		            byte state = armControlState.stateType();
		            
		            if(state == grl.flatbuffer.ArmState.ShutdownArm){
		            	stop = true;
		            }
		            else if (state == grl.flatbuffer.ArmState.MoveArmTrajectory) {
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
		            } else if (state == grl.flatbuffer.ArmState.MoveArmJointServo) {
		            	if (currentMotion != null) {
		            		currentMotion.cancel();
		            	}
		            	tm.setActive(false);
		            	
		            	
		            	
		            	grl.flatbuffer.MoveArmServo mas = null;
		            	armControlState.state(mas);
		            	
		            	if(_currentActiveConfiguration.commandInterface() == grl.flatbuffer.KUKAiiwaInterface.SmartServo){
		            		grl.flatbuffer.JointState jointState = mas.goal();
			            	
			            	for (int k = 0; k < destination.getAxisCount(); ++k)
			                {
			                	destination.set(k, jointState.position(k));
			                }
			                theSmartServoRuntime.setDestination(destination);
		            		
		            	} else if(_currentActiveConfiguration.commandInterface()==grl.flatbuffer.KUKAiiwaInterface.FRI){
	
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
		            	
		            } else if (state == grl.flatbuffer.ArmState.StopArm) {
		            	theSmartServoRuntime.stopMotion();
		            	if (currentMotion != null) {
		            		currentMotion.cancel();
		            	}
		            	tm.setActive(false);
		            	
		            } else if (state == grl.flatbuffer.ArmState.TeachArm) {
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
		            newCommand = false;
	            }
        }}
            
        
        // done
        ucr.stop();
        subscriber.close();
        context.term();
        try {
			updateConfigThread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        friSession.close();
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
