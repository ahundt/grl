package grl;

import java.lang.Runnable;
import java.nio.ByteBuffer;

import org.zeromq.ZMQ;

import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.userInterface.smartServo.ServoMotionUtilities;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.Tool;

public class UpdateConfiguration {

	private LBR  _lbr;
	private Tool _flangeAttachment;
	private FRIConfiguration _FRIConfiguration = null;
	public LBR get_lbr() {
		return _lbr;
	}

	public void set_lbr(LBR _lbr) {
		this._lbr = _lbr;
	}

	public Tool get_flangeAttachment() {
		return _flangeAttachment;
	}

	public void set_flangeAttachment(Tool _flangeAttachment) {
		this._flangeAttachment = _flangeAttachment;
	}

	public FRIConfiguration get_FRIConfiguration() {
		return _FRIConfiguration;
	}

	public void set_FRIConfiguration(FRIConfiguration _FRIConfiguration) {
		this._FRIConfiguration = _FRIConfiguration;
	}

	public FRISession get_FRISession() {
		return _FRISession;
	}

	public void set_FRISession(FRISession _FRISession) {
		this._FRISession = _FRISession;
	}

	private FRISession _FRISession = null;

	public UpdateConfiguration(LBR lbr, Tool flangeAttachment){
		_lbr = lbr;
		_flangeAttachment = flangeAttachment;
	}

	/**
	 * Create an End effector tool
	 * @param link the Flatbuffer representing the tool
	 * @return the Kuka tool object
	 */
	public PhysicalObject  linkObjectToPhysicalObject(grl.flatbuffer.LinkObject link){
		if(link == null) return null;

        grl.flatbuffer.Vector3d toolpos = link.pose().position();
        grl.flatbuffer.Quaternion toolrot = link.pose().orientation();
        grl.flatbuffer.Vector3d compos = link.inertia().pose().position();
        grl.flatbuffer.Quaternion comrot = link.inertia().pose().orientation();


        double[] translationOfTool =
        { toolpos.x(), toolpos.y(), toolpos.z() };
        double[] centerOfMassInMillimeter =
        { comrot.x(), comrot.y(), comrot.z() };

        return ServoMotionUtilities.createTool(_lbr,
        		link.name(), translationOfTool,
        		link.inertia().mass(),
                centerOfMassInMillimeter);
	}

	/**
	 * Sets the FRI Configuration and FRI Session stored by the UpdateConfiguration object
	 * @param friConfigBuf flatbuffer containing fri settings
	 * @param controllingLaptopIPAddress
	 * @return error code true if there was a problem, success code false otherwise
	 */
	public boolean FRIflatbufferToConfigAndSession (grl.flatbuffer.FRI friConfigBuf, String controllingLaptopIPAddress ){
		if(friConfigBuf == null) return true;
        // Configure and start FRI session
        FRIConfiguration _FRIConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, controllingLaptopIPAddress);

        if(friConfigBuf !=null && friConfigBuf.sendPeriodMillisec() >0){
        	_FRIConfiguration.setSendPeriodMilliSec(friConfigBuf.sendPeriodMillisec());
			/**
			 * Defines the answer rate factor (type: int)
			 * The answer rate factor defines the rate at which the external
             * system is to send data to the robot controller:
			 * Answer rate = answer rate factor*send rate
			 */

        	_FRIConfiguration.setReceiveMultiplier(friConfigBuf.setReceiveMultiplier());
        }

        _FRISession = new FRISession(_FRIConfiguration);

        return false;
	}


}
