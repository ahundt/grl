package grl.zmqDriver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

public class TeachMode implements Runnable {

	private JointImpedanceControlMode mode;
	private LBR lbr;
	private boolean active;
	
	public TeachMode(LBR lbr) {
		mode = new JointImpedanceControlMode(100,100,100,100,100,100,100);
		mode.setDampingForAllJoints(0.5);
		mode.setStiffnessForAllJoints(100);
		active = false;
	}
	
	public synchronized void setActive(boolean active) {
		this.active = active;
	}
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		synchronized (this) {
			if (active) {
				lbr.move(positionHold(mode, 10, TimeUnit.MILLISECONDS));
			}
		}
	}

}
