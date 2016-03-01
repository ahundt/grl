package grl.driver;



import grl.TeachMode;

import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.recovery.IRecovery;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class TeachModeTest extends RoboticsAPIApplication {
	private Controller _lbrController;
	private LBR _lbr;
	private Tool gripper;
	//private IRecovery _pausedApplicationRecovery = null;

	public void initialize() {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
		gripper = getApplicationData().createFromTemplate("FlangeAttachment");
        //_pausedApplicationRecovery = getRecovery();
	}

	public void run() {
        boolean useTeachModeObject = false;

        //boolean isRecoveryRequired = _pausedApplicationRecovery.isRecoveryRequired();
		if(useTeachModeObject){
			/// @todo This section isn't working... debug it to fix the object
	        
	        TeachMode tm = new TeachMode(_lbr);
	        Thread tmThread = new Thread(tm);
	        
	        // Receive Flat Buffer and Move to Position
	        // TODO: add a message that we send to the driver with data log strings
	        tm.setActive(true);
	        tmThread.start();
	        boolean stop = false;
	        while (!stop) {
	        	//isRecoveryRequired = _pausedApplicationRecovery.isRecoveryRequired();
	        	// how do we check if the user wants to stop the app?
	        }

	    	tm.setActive(false);
			
		}
    	else 
    	{
    		JointImpedanceControlMode mode = new JointImpedanceControlMode(100,100,100,100,100,100,100);
    		mode.setDampingForAllJoints(0.5);
    		mode.setStiffnessForAllJoints(100);

    		while(!useTeachModeObject) {

    			_lbr.move(positionHold(mode, 10, TimeUnit.MILLISECONDS));

    		}
    	}
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		TeachModeTest app = new TeachModeTest();
		app.runApplication();
	}
}
