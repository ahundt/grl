package friCommunication;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRIJointOverlay;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;

/**
 * Creates a FRI Session.
 */
public class FRI_Command extends RoboticsAPIApplication
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
        friConfiguration.setSendPeriodMilliSec(50);
        FRISession friSession = new FRISession(friConfiguration);
		FRIJointOverlay motionOverlay = new FRIJointOverlay(friSession);
		
        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {

        }

        // move to start pose
        _lbr.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(90), .0));
        
	     for(int i = 0; i < 100; i++) {
	         // async move with overlay ...
	         _lbr.moveAsync(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(90), .0)
	                 .setJointVelocityRel(0.2)
	                 .addMotionOverlay(motionOverlay)
	                 .setBlendingRel(0.1)
	                 );

	         // ... blending into sync move with overlay
	         _lbr.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
	                 .setJointVelocityRel(0.2)
	                 .addMotionOverlay(motionOverlay)
	                 );
	     }

	    _lbr.move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0));
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
        final FRI_Command app = new FRI_Command();
        app.runApplication();
    }

}
