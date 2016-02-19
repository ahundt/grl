package grl.zmqDriver;

import grl.flatbuffer.Quaternion;
import grl.flatbuffer.Vector3d;
import grl.flatbuffer.kuka.iiwa.KUKAiiwaArmConfiguration;

import java.lang.Runnable;
import java.nio.ByteBuffer;

import org.jeromq.ZMQ.Socket;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

public class UpdateConfigurationRunnable implements Runnable {

    private PhysicalObject _toolAttachedToLBR;
	private Socket sub;
	private LBR _lbr;
	private boolean stop;
	
	public UpdateConfigurationRunnable(Socket socket, LBR lbr, PhysicalObject tool) {
		sub = socket;
		_toolAttachedToLBR = tool;
		_lbr = lbr;
		stop = false;
	}
	
	public void stop() {
		synchronized(_lbr) {
			stop = true;
		}
	}
	
	@Override
	public void run() {
		while (!stop) {
	        byte [] data = sub.recv();
	        ByteBuffer bb = ByteBuffer.wrap(data);

	        KUKAiiwaArmConfiguration conf = KUKAiiwaArmConfiguration.getRootAsKUKAiiwaArmConfiguration(bb);
	        
	        Vector3d toolpos = conf.tools(0).pose().position();
	        Quaternion toolrot = conf.tools(0).pose().orientation();
	        Vector3d compos = conf.tools(0).inertia().pose().position();
	        Quaternion comrot = conf.tools(0).inertia().pose().orientation();

	        double[] translationOfTool =
	        { toolpos.x(), toolpos.y(), toolpos.z() };
	        double[] centerOfMassInMillimeter =
	        { comrot.x(), comrot.y(), comrot.z() };
	        
	        synchronized(_lbr) {
		        _toolAttachedToLBR = ServoMotionUtilities.createTool(_lbr,
		        		conf.tools(0).name(), translationOfTool,
		        		conf.tools(0).inertia().mass(),
		                centerOfMassInMillimeter);
	        }
		}
	}

}
