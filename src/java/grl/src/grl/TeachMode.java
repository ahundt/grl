package grl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.MMCMotions.handGuiding;

import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.task.ITaskLogger;

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

	private LBR lbr;	

	IMotionContainer _handGuidingMotionContainer = null;
	HandGuidingMotion _handGuidingMotion;
	private double[] _maxAllowedJointLimits;
	private double[] _minAllowedJointLimits;
	Tool _flangeAttachment;
	JointImpedanceControlMode _teachControlMode;
	private static volatile boolean useHandGuidingMotion;
	private static volatile boolean isEnableEnded;
	private static volatile boolean stop = false;
	ITaskLogger _logger = null;
	int iter = 0;

	/**
	 * 
	 * @param flangeAttachment
	 * @param maxAllowedJoints
	 * @param minAllowedJoints
	 */
	public TeachMode(Tool flangeAttachment, double[] maxAllowedJoints, double[] minAllowedJoints) {
		_handGuidingMotion = handGuiding();
		_maxAllowedJointLimits = maxAllowedJoints;
		_minAllowedJointLimits = minAllowedJoints;
		_flangeAttachment = flangeAttachment;
		useHandGuidingMotion = false;
	}

	public void setLogger(ITaskLogger logger) {
		_logger = logger;
	}

	/**
	 * 
	 * @param teachModeControlMode
	 */
	public void setTeachModeControlMode(JointImpedanceControlMode teachModeControlMode) {
		_teachControlMode = teachModeControlMode;
	}

	public void enable() {
		synchronized (this) {

			useHandGuidingMotion = true;
		}
	}
	
	/**
	 * Tell the hand guiding motion (teach mode) to stop
	 * and the thread exits. Make sure 
	 * isEnableEnded returns true before calling this!
	 * @return false on failure; true on successfully starting the shutdown process
	 */
	public synchronized boolean stop(){
		if(!isEnableEnded) return false;
		stop = true;
		return true;
	}
	
	
	/**
	 * 
	 */
	public boolean cancel(){
		//warn("Cancel received");
		
		synchronized (this) {

			useHandGuidingMotion = false;
			
//			if (_handGuidingMotionContainer != null) {
//				_handGuidingMotionContainer.cancel();
//				_handGuidingMotionContainer = null;
//				return true;
//			}
		}
		return true;
		//warn("Cancel complete");
	}
	
	/**
	 * @brief true there are no outstanding calls made to enable() and you can switch modes
	 * 
	 * @see enable()
	 * 
	 * The sunrise HandGuidingMotion mode has an API
	 * quirk where you need to push the physical button
	 * to end the hand guiding mode. This tells you if
	 * the motion is both over and the button has been
	 * pushed so the action is really truly complete.
	 * @return true if you can switch modes now, false if someone still needs to press the physical button
	 */
	public synchronized boolean isEnableEnded(){
		return isEnableEnded;
	}

	/**
	 * This only prints if the logger has been set!
	 * @param str
	 */
	private void warn(String str) {
		if (_logger != null) {
			_logger.warn(str);
		}
	}

	//@Override
	public void run() {

		// trying to use kuka's provided handguidingmotion but it isn't working now.
		// using an if statement to default to old behavior.
		synchronized(this) {
			useHandGuidingMotion = false;
		}
		_handGuidingMotionContainer = null;
		
		while(!stop) {
			//warn("Starting new hand guiding motion");

			try {

				// see kuka documentation 1.9 for details
				synchronized(this) {

					if (!useHandGuidingMotion) {
						//warn("breaking hand guiding motion");
						try {
							if(!isEnableEnded && _logger!=null)
							{
								_logger.info("Teach Mode and HandGuidingMotion Complete!");
							}
							isEnableEnded = true;
							this.wait(100);
						} catch (InterruptedException e) {
							warn("Interruption in TeachMode: " + e.toString());
						}
						continue;
					}
					
					warn("creating hand guiding motion " + useHandGuidingMotion);
					isEnableEnded = false;
					_handGuidingMotion = handGuiding()
							.setAxisLimitsMax(_maxAllowedJointLimits)
							.setAxisLimitsMin(_minAllowedJointLimits)
							.setAxisLimitsEnabled(true, true, true, true, true, true, true)
							.setAxisLimitViolationFreezesAll(false).setPermanentPullOnViolationAtStart(true);
					isEnableEnded = false;
				}
				//warn("hand guiding motion moving... " + useHandGuidingMotion);
				//if (_handGuidingMotionContainer == null || _handGuidingMotionContainer.isFinished()) {
				_handGuidingMotionContainer = _flangeAttachment.move(_handGuidingMotion);
				//}
				//warn("done hand guiding");

			} catch (CommandInvalidException e) {
				_handGuidingMotionContainer = null;
				warn(e.toString());
			} catch (IllegalStateException e) {
				_handGuidingMotionContainer = null;
				warn(e.toString());
			}
		}

		warn("Teach Mode Thread Exiting");
	}

}
