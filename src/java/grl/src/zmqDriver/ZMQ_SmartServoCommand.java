package zmqDriver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.nio.ByteBuffer;

import org.jeromq.ZMQ;
import org.jeromq.ZMQ.Context;
import org.jeromq.ZMQ.Socket;

import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;

import flatbuffers.JointState;

/**
 * Creates a FRI Session.
 */
public class ZMQ_SmartServoCommand extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _hostName;
    private String _kukaIPAddress;

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
        _kukaIPAddress = "tcp://172.31.1.147:5563";
        
    }

    @Override
    public void run()
    {
        // Configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _hostName);
        friConfiguration.setSendPeriodMilliSec(4);
        FRISession friSession = new FRISession(friConfiguration);
        
       // Prepare ZeroMQ context and dealer
        Context context = ZMQ.context(1);
        Socket subscriber = context.socket(ZMQ.DEALER);
        subscriber.connect(_kukaIPAddress);
        subscriber.setRcvHWM(1000000);
        
        // Receive Flat Buffer and Move to Position
        // TODO: make into while loop and add exception to exit loop
        for (int i=0; i<100; i++) {
            byte [] data = subscriber.recv();
            ByteBuffer bb = ByteBuffer.wrap(data);
            JointState jointState = JointState.getRootAsJointState(bb);
            // TODO: Test move command
            _lbr.moveAsync(ptp(jointState.position(0), jointState.position(1), jointState.position(2), jointState.position(3), jointState.position(4), jointState.position(5), jointState.position(6)));
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
