package grl.zmqDriver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.nio.ByteBuffer;

import org.jeromq.ZMQ;
import org.jeromq.ZMQ.Context;
import org.jeromq.ZMQ.Socket;

import com.google.flatbuffers.Table;
import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

import grl.flatbuffer.ArmConfiguration;
import grl.flatbuffer.JointState;
import grl.flatbuffer.ArmControlState;
import grl.flatbuffer.ArmState;
import grl.flatbuffer.MoveArmJoints;
import grl.flatbuffer.MoveArmServo;
import grl.flatbuffer.MoveArmTrajectory;
import grl.flatbuffer.Quaternion;
import grl.flatbuffer.Vector3d;

/**
 * Creates a FRI Session.
 */
public class ZMQ_SmartServoCommand extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _hostName;
    private String _controllingLaptopIPAddress;
    private PhysicalObject _toolAttachedToLBR;
    private ISmartServoRuntime theSmartServoRuntime = null;
	private String _controllingLaptopIPAddressConfig;

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
        // Configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _hostName);
        friConfiguration.setSendPeriodMilliSec(4);
        FRISession friSession = new FRISession(friConfiguration);
        
        // TODO: remove default start pose
        // move do default start pose
        //_toolAttachedToLBR.move(ptp(Math.toRadians(10), Math.toRadians(10), Math.toRadians(10), Math.toRadians(-90), Math.toRadians(10), Math.toRadians(10),Math.toRadians(10)));

        
       // Prepare ZeroMQ context and dealer
        Context context = ZMQ.context(1);
        Socket subscriber = context.socket(ZMQ.DEALER);
        subscriber.connect(_controllingLaptopIPAddress);
        subscriber.setRcvHWM(1000000);
        
        Socket configSubscriber = context.socket(ZMQ.DEALER);
        configSubscriber.connect(_controllingLaptopIPAddressConfig);
        configSubscriber.setRcvHWM(1000000);

        getArmConfiguration(configSubscriber);

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
        
        byte [] data = subscriber.recv();
        ByteBuffer bb = ByteBuffer.wrap(data);

        ArmControlState armControlState = ArmControlState.getRootAsArmControlState(bb);
        IMotionContainer currentMotion = null;
        
        // Receive Flat Buffer and Move to Position
        // TODO: make into while loop and add exception to exit loop
        // TODO: add a message that we send to the driver with data log strings
        // TODO: add a message we receive (and send) that tells the other application to stop running
        for (int i=0; i<100000; i++) {
        	// TODO: IMPORTANT: this recv call must be made asynchronous
            data = subscriber.recv();
            bb = ByteBuffer.wrap(data);

            armControlState = ArmControlState.getRootAsArmControlState(bb);
            long num = armControlState.sequenceNumber();
            double time = armControlState.timeStamp();
            byte state = armControlState.stateType();
            
            if (state == ArmState.MoveArmTrajectory) {
            	tm.setActive(false);
            	theSmartServoRuntime.stopMotion();
            	if (currentMotion != null) {
            		currentMotion.cancel();
            	}
            	
            	MoveArmTrajectory maj = null;
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
            } else if (state == ArmState.MoveArmServo) {
            	if (currentMotion != null) {
            		currentMotion.cancel();
            	}
            	tm.setActive(false);
            	
            	MoveArmServo mas = null;
            	armControlState.state(mas);
            	
            	JointState jointState = mas.goal();

            	for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                	destination.set(k, jointState.position(k));
                }
                theSmartServoRuntime.setDestination(destination);
            	
            } else if (state == ArmState.StopArm) {
            	theSmartServoRuntime.stopMotion();
            	if (currentMotion != null) {
            		currentMotion.cancel();
            	}
            	tm.setActive(false);
            	
            } else if (state == ArmState.TeachArm) {
            	theSmartServoRuntime.stopMotion();
            	if (currentMotion != null) {
            		currentMotion.cancel();
            	}
            	tm.setActive(true);
            }

            //theSmartServoRuntime.setDestination(destination);
            // TODO: test to make sure the accelerations are present
            //double[] acceleration = {jointState.acceleration(0),jointState.acceleration(1),jointState.acceleration(2),jointState.acceleration(3),jointState.acceleration(4),jointState.acceleration(5),jointState.acceleration(6)};

            // TODO: Test move command
            //_lbr.moveAsync(ptp(jointState.position(0), jointState.position(1), jointState.position(2), jointState.position(3), jointState.position(4), jointState.position(5), jointState.position(6)));
            		//.setJointAccelerationRel(acceleration));
        }
        
        // done
        subscriber.close();
        context.term();
        System.exit(1);
        friSession.close();
    }

    private void getArmConfiguration(Socket sub) {
        byte [] data = sub.recv();
        ByteBuffer bb = ByteBuffer.wrap(data);

        ArmConfiguration conf = ArmConfiguration.getRootAsArmConfiguration(bb);
        
        Vector3d toolpos = conf.tool().pose().position();
        Quaternion toolrot = conf.tool().pose().orientation();
        Vector3d compos = conf.tool().centerOfMass().position();
        Quaternion comrot = conf.tool().centerOfMass().orientation();

        double[] translationOfTool =
        { toolpos.x(), toolpos.y(), toolpos.z() };
        double[] centerOfMassInMillimeter =
        { comrot.x(), comrot.y(), comrot.z() };
        
        _toolAttachedToLBR = ServoMotionUtilities.createTool(_lbr,
        		conf.tool().name(), translationOfTool,
        		conf.tool().mass(),
                centerOfMassInMillimeter);
	}

	/**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final ZMQ_SmartServoCommand app = new ZMQ_SmartServoCommand();
        app.runApplication();
    }

}
