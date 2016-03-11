package grl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * 
 * @todo This object and section isn't working... debug it in TeachModeTest to fix the object
 * 
 * In TeachModeTest you will need to set 
 *       boolean useTeachModeObject = false;
 *       
 * This object is supposed to make it easy to enable/disable teach 
 * mode with a single call to start() and stop() when you want to
 * enable and disable teach mode.
 *  
 *  @todo the method used to simulate teach mode is a little jumpy, needs to be improved.
 * 
 * @author Andrew Hundt
 *
 */
public class TeachMode implements Runnable {

	private JointImpedanceControlMode mode;
	private LBR lbr;
	private boolean active;
	//private Thread thread;
	
	public TeachMode(LBR lbr) {
		//thread = new Thread(this);
		mode = new JointImpedanceControlMode(100,100,100,100,100,100,100);
		mode.setDampingForAllJoints(0.5);
		mode.setStiffnessForAllJoints(100);
		active = false;
	}
	
	public void setActive(boolean active) {
		this.active = active;
		//if(active) thread.start();
	}
	
	public boolean getActive(){
		return this.active;
	}
	
	public void stop(){
		active = false;
	}
	
	//@Override
	public void run() {
		// TODO Auto-generated method stub
		boolean local_active = true;
		//synchronized (this) {
		active = true;
		//}
		while(local_active) {

			//synchronized (this) {
				local_active = active;
			//}
				
			lbr.move(positionHold(mode, 10, TimeUnit.MILLISECONDS));

		}
	}
	
	public void start(){
			active = true;
	}

}
