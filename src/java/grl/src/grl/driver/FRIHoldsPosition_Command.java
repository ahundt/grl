package grl.driver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import grl.FRIMode;
import grl.ProcessDataManager;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation.FRISessionState;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.EmergencyStop;
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
	//private FRIJointOverlay  _motionOverlay = null;
	private FRIMode _FRIModeRunnable = null;
	private Thread _FRIModeThread = null;
	private ProcessDataManager _processDataManager = null;

    @Override
    public void initialize()
    {

		_processDataManager = new ProcessDataManager(this);
	
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _hostName = _processDataManager.get_FRI_KONI_LaptopIPAddress();
    }

    @Override
    public void run()
    {

		int message_counter = 0;

		_FRIModeRunnable = new FRIMode(
				_lbr,_hostName,4);

		_FRIModeRunnable.setLogger(getLogger());
		_FRIModeThread = new Thread(_FRIModeRunnable);
		_FRIModeThread.start();
		
		 PositionControlMode controlMode = new PositionControlMode();
		while( _lbr.getSafetyState().getEmergencyStopInt()==EmergencyStop.INACTIVE)
		{
			
			if(!_FRIModeRunnable.isCommandingWaitOrActive())    
			{
				    if(message_counter % 1000 == 0) getLogger().info("FRI MotionOverlay starting...");
				    controlMode = new PositionControlMode();
				    _lbr.move(positionHold(controlMode, 100, TimeUnit.MILLISECONDS));
					_FRIModeRunnable.setControlMode(controlMode);
					_FRIModeRunnable.enable();
			}

			if(message_counter % 1000 == 0) getLogger().info("FRI MotionOverlay Quality Sample: " + _FRIModeRunnable.getQualityString());
		    message_counter++;
		}
		_FRIModeRunnable.stop();
        // done
        //_friSession.close();
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
