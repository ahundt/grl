package grl.friCommunication;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRIJointOverlay;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * Creates a FRI Session.
 */
public class FRIHoldsPosition_Command extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _hostName;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _hostName = "192.170.10.100";
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _hostName);
        friConfiguration.setSendPeriodMilliSec(4);
        FRISession friSession = new FRISession(friConfiguration);
		FRIJointOverlay motionOverlay = new FRIJointOverlay(friSession);
		
		 try {
			friSession.await(10, TimeUnit.SECONDS);
		} catch (TimeoutException e) {
			// TODO Automatisch generierter Erfassungsblock
			e.printStackTrace();
			friSession.close();
			return;
		}
		//CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();
		//controlMode.parametrize(CartDOF.X).setStiffness(500.0);
		//controlMode.parametrize(CartDOF.ALL).setDamping(0.7);
//		 JointImpedanceControlMode controlMode = new JointImpedenceControlMode();
//		 controlMode.setDampingForAllJoints(0.7);
//		 controlMode.setStiffnessForAllJoints(500);
		 PositionControlMode controlMode = new PositionControlMode();
        // TODO: remove default start pose
        // move to default start pose
        _lbr.move(ptp(Math.toRadians(10), Math.toRadians(10), Math.toRadians(10), Math.toRadians(-90), Math.toRadians(10), Math.toRadians(10),Math.toRadians(10)));

        // sync move for infinite time with overlay ...
		_lbr.move(positionHold(controlMode, -1, TimeUnit.SECONDS).addMotionOverlay(motionOverlay));
        //_lbr.moveAsync(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0));
        
        // TODO: remove default start pose
        // move to default start pose
        _lbr.move(ptp(Math.toRadians(10), Math.toRadians(10), Math.toRadians(10), Math.toRadians(-90), Math.toRadians(10), Math.toRadians(10),Math.toRadians(10)));

        // done
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
        final FRIHoldsPosition_Command app = new FRIHoldsPosition_Command();
        app.runApplication();
    }

}
