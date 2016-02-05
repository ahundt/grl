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
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

import grl.flatbuffer.JointState;
import grl.flatbuffer.ArmControlState;
import grl.flatbuffer.ArmState;
import grl.flatbuffer.MoveArmJoints;

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

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _hostName = "192.170.10.100";
        // **********************************************************************
        // *** change next line to the KUKA address and Port Number           ***
        // **********************************************************************
        _controllingLaptopIPAddress = "tcp://172.31.1.100:30010";
        

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
        _toolAttachedToLBR.move(ptp(Math.toRadians(10), Math.toRadians(10), Math.toRadians(10), Math.toRadians(-90), Math.toRadians(10), Math.toRadians(10),Math.toRadians(10)));

        
       // Prepare ZeroMQ context and dealer
        Context context = ZMQ.context(1);
        Socket subscriber = context.socket(ZMQ.DEALER);
        subscriber.connect(_controllingLaptopIPAddress);
        subscriber.setRcvHWM(1000000);
     
       

        JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());
        SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 20% of systems abilities
        aSmartServoMotion.setJointAccelerationRel(0.2);
        aSmartServoMotion.setJointVelocityRel(0.2);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

        //_toolAttachedToLBR.getDefaultMotionFrame().moveAsync(aSmartServoMotion);
        
        // Fetch the Runtime of the Motion part
        theSmartServoRuntime = aSmartServoMotion.getRuntime();
        
     	// create an JointPosition Instance, to play with
        JointPosition destination = new JointPosition(
                _lbr.getJointCount());
        
        
        byte [] data = subscriber.recv();
        ByteBuffer bb = ByteBuffer.wrap(data);

        ArmControlState armControlState = ArmControlState.getRootAsArmControlState(bb);
        

        // move to start pose
        //_lbr.move(ptp(jointState.position(0), jointState.position(1), jointState.position(2), jointState.position(3), jointState.position(4), jointState.position(5), jointState.position(6)));
		//.setJointAccelerationRel(acceleration));

 
        
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
            
            if (state == ArmState.MoveArmJoints) {
            	MoveArmJoints maj = null;
            	armControlState.state(maj);
            	

	            for (int j = 0; j < maj.trajLength(); j++) {
	            	
	            	JointPosition pos = new JointPosition(_lbr.getCurrentJointPosition());
	            	
		            for (int k = 0; k < destination.getAxisCount(); ++k)
		            {
		            	//destination.set(k, maj.traj(j).position(k));
		            	pos.set(k, maj.traj(j).position(k));
		            }
		        	_lbr.moveAsync(new SmartServo(pos));
		            
	            }
            }

            theSmartServoRuntime.setDestination(destination);
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
