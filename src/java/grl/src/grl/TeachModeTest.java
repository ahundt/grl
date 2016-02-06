package grl;


import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

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
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private Tool gripper;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
		gripper = getApplicationData().createFromTemplate("Gripper");
	}

	public void run() {
		//lbr.move(ptpHome());

		//gripper.attachTo(lbr.getFlange());
		

		double ximp = getApplicationData().getProcessData("ximp").getValue();
		double yzimp = getApplicationData().getProcessData("yzimp").getValue();
		double rimp = getApplicationData().getProcessData("rimp").getValue();
		double xdamp = getApplicationData().getProcessData("xdamp").getValue();
		double yzdamp = getApplicationData().getProcessData("yzdamp").getValue();
		double rdamp = getApplicationData().getProcessData("rdamp").getValue();
		
		CartesianImpedanceControlMode mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.X).setStiffness(ximp).setDamping(xdamp);
		mode.parametrize(CartDOF.Y, CartDOF.Z).setStiffness(yzimp).setDamping(yzdamp);
		mode.parametrize(CartDOF.A,CartDOF.B,CartDOF.C).setStiffness(rimp).setDamping(rdamp);
		
		for(int i = 0; i < 1000000; ++i) {
			lbr.move(positionHold(mode, 1, TimeUnit.SECONDS));
		}
		
		
		//lbr.move(ptp(getApplicationData().getFrame("/BlueGear/CenterPrePos")));
		//lbr.move(positionHold(mode, -30, TimeUnit.SECONDS));
		//lbr.move(ptp(getApplicationData().getFrame("/BlueGear/CenterPrePos")));
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		TeachModeTest app = new TeachModeTest();
		app.runApplication();
	}
}
