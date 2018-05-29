package grl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
//uncomment for sunrise OS 1.9 and below
//import static com.kuka.roboticsAPI.motionModel.MMCMotions.handGuiding;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation.FRISessionState;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.task.ITaskLogger;

/**
 *
 * Activate FRI mode in a separate thread
 *
 */
 // Provide a Runnable object. The Runnable interface defines a single method, run, meant to contain the code executed in the thread.
 // The Runnable object is passed to the Thread constructor,
public class FRIMode implements Runnable {

	private LBR _lbr;



	private AbstractMotionControlMode _activeMotionControlMode;
	private FRISession       _friSession = null;
	private FRIJointOverlay  _motionOverlay = null;
	private FRIConfiguration _friConfiguration = null;
	private String _hostName = null;
	private volatile boolean useHandGuidingMotion;
	private volatile boolean isEnableEnded;
	private volatile boolean stop = false;
    private volatile boolean timedOut = false;

	IMotionContainer currentMotion = null;
	ITaskLogger _logger = null;
	int iter = 0;

	/**
	 *
	 * @param flangeAttachment
	 * @param maxAllowedJoints
	 * @param minAllowedJoints
	 */
	public FRIMode(LBR lbr, String hostName, int sendPeriodMillisec) {
		_lbr = lbr;
		_hostName = hostName;
		useHandGuidingMotion = false;
		stop = false;
		timedOut = false;

        _friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _hostName);
        _friConfiguration.setSendPeriodMilliSec(sendPeriodMillisec);
		/**
		* Using an instance of the class FRISession, the FRI connection between the
        * robot controller and external system is opened and Monitor mode is automatically activated.
		* If Monitor mode is activated, the connection quality is continuously determined
        * and evaluated.
		*/
        if(_friSession == null) _friSession = new FRISession(_friConfiguration);
		//_motionOverlay = new FRIJointOverlay(_friSession);
	}

	public void setLogger(ITaskLogger logger) {
		_logger = logger;
	}

	public boolean isCommandingWaitOrActive()
	{
		boolean ret;
		/**
		* The method getFRIChannelInformation() can be used to poll the following information, the FRI connection and FRI state,
		* and save it in a variable of type FRIChannelInformation.
		* Type: FRIChannelInformation
		*/

		synchronized (this) {
			ret = _friSession.getFRIChannelInformation().getFRISessionState().compareTo(FRISessionState.COMMANDING_ACTIVE) != 0
					&& _friSession.getFRIChannelInformation().getFRISessionState().compareTo(FRISessionState.COMMANDING_WAIT) != 0;
		}

		return !ret;
	}

	public String getQualityString()
	{
		return _friSession.getFRIChannelInformation().getQuality().toString();
	}

	/**
	 *
	 * @param setControlMode
	 */
	public void setControlMode(AbstractMotionControlMode teachModeControlMode) {

		synchronized (this) {
			_activeMotionControlMode = teachModeControlMode;
		}
	}

	public void setFRIConfiguration(FRIConfiguration friConfguration){
		_friConfiguration = friConfguration;
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

		if(currentMotion !=null) currentMotion.cancel();

		return true;
	}

	public synchronized boolean isTimedOut(){
	   return timedOut;
	}

	/**
	 *
	 * Cancel the motion, thread stays in existence.
	 */
	public boolean cancel(){
		//warn("Cancel received");

		synchronized (this) {

			useHandGuidingMotion = false;

			if(currentMotion !=null) currentMotion.cancel();
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
		_motionOverlay = null;

		while(!stop && !timedOut) {
			//warn("Starting new hand guiding motion");

			try {

				// see kuka documentation 1.9 for details
				synchronized(this) {
					if(!timedOut){
					if (!useHandGuidingMotion) {
						//warn("breaking hand guiding motion");
						try {
							if(!isEnableEnded && _logger!=null)
							{
								_logger.info("FRI Joint Overlay Complete!");
							}
							isEnableEnded = true;
							this.wait(100);
						} catch (InterruptedException e) {
							warn("Interruption in FRIMode: " + e.toString());
						}
						continue;
					}

					warn("creating FRI Joint Overlay " + useHandGuidingMotion);
					isEnableEnded = false;
					_motionOverlay = new FRIJointOverlay(_friSession);
//					_handGuidingMotion = handGuiding()
//							.setAxisLimitsMax(_maxAllowedJointLimits)
//							.setAxisLimitsMin(_minAllowedJointLimits)
//							.setAxisLimitsEnabled(true, true, true, true, true, true, true)
//							.setAxisLimitViolationFreezesAll(false).setPermanentPullOnViolationAtStart(true);
					isEnableEnded = false;
				}
				}
				//warn("hand guiding motion moving... " + useHandGuidingMotion);
				//if (_handGuidingMotionContainer == null || _handGuidingMotionContainer.isFinished()) {
				if(     !timedOut
						&& _friSession.getFRIChannelInformation().getFRISessionState().compareTo(FRISessionState.COMMANDING_ACTIVE) != 0
						&& _friSession.getFRIChannelInformation().getFRISessionState().compareTo(FRISessionState.COMMANDING_WAIT) != 0)
					{
						_logger.info("FRI Joint Overlay starting...");

						try {
							_friSession.await(10, TimeUnit.SECONDS);

							_motionOverlay = new FRIJointOverlay(_friSession);
							currentMotion = _lbr.move(positionHold(_activeMotionControlMode, -1, TimeUnit.SECONDS).addMotionOverlay(_motionOverlay));
							_logger.info("FRI Joint Overlay ended...");

						} catch (TimeoutException e) {
							//_logger.error("FRISession timed out, closing...");
							//e.printStackTrace();
							//_friSession.close();
							_logger.error("FRIMode: FRISession timed out, need to restart from scratch...");

							if(currentMotion !=null) currentMotion.cancel();
							//_friSession.close();
							//timedOut = true;
							//e.printStackTrace();
							//_friSession.close();
							return;
						}
					}

				//}
				//warn("done hand guiding");

			} catch (CommandInvalidException e) {
				currentMotion = null;
				//_handGuidingMotionContainer = null;
				warn(e.toString());
			} catch (IllegalStateException e) {
				currentMotion = null;
				//_handGuidingMotionContainer = null;
				warn(e.toString());
			}
		}

		warn("FRI Mode Thread Exiting");
	}

}
