/**
 * Copyright (c) 2017, Dinesh Thakur
 * University of Pennsylvania
 *
 * Copyright (c) 2015, Andrew Hundt
 *
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

package grl.driver;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import grl.FRIMode;
import grl.ProcessDataManager;
import grl.StartStopSwitchUI;
import grl.TeachMode;
import grl.UpdateConfiguration;
import grl.UDPManager;
import grl.flatbuffer.ArmState;
import grl.flatbuffer.KUKAiiwaInterface;
import grl.flatbuffer.KUKAiiwaMonitorState;
import grl.flatbuffer.KUKAiiwaState;
import grl.flatbuffer.KUKAiiwaStates;
import grl.flatbuffer.MoveArmJointServo;
import grl.flatbuffer.MoveArmTrajectory;
import grl.flatbuffer.Wrench;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.google.flatbuffers.FlatBufferBuilder;
import com.google.flatbuffers.Table;

// 1. Compiler error here? COMMENT ALL FLEXFELLOW AND MEDIAFLANGEIOGROUP lines
// 2. FlexFellow lines commented? DO NOT DELETE THEM, other people have different hardware!
// 3. Have a FlexFellow? Uncomment all the FlexFellow lines and the lights will let you know
//    what mode the arm is in!
//
//import com.kuka.generated.ioAccess.FlexFellowIOGroup;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation.FRISessionState;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.ServoMotionJP;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
//import com.kuka.generated.ioAccess.FlexFellowIOGroup;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.recovery.IRecovery;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.EmergencyStop;
import com.kuka.roboticsAPI.deviceModel.JointLimits;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.executionModel.IExecutionContainer;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainerListener;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.HandGuidingControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;

// uncomment for sunrise OS 1.9 and below
// import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;

/**
 * Creates a FRI Session.
 */
public class GRL_Driver extends RoboticsAPIApplication
{
	private ProcessDataManager _processDataManager = null; // Stores variables that can be modified by teach pendant in "Process Data" Menu
	private Controller _lbrController;
	private LBR _lbr;
	private StartStopSwitchUI _startStopUI = null;

	/// The interface on which commands are being sent to the robot controller (this program),
	/// includes SmartServo, DirectServo, and FRI
	private byte _commandInterface = KUKAiiwaInterface.SmartServo;

	/// The interface on which monitor data is being sent to the Laptop (C++ program over network)
	/// This can be FRI (udp packets over KONI port for monitoring state) (works by default)
	/// or SmartServo/DirectServo (JAVA UDP interface will be used to send position/torques back (not yet implemented)
	private byte _monitorInterface = KUKAiiwaInterface.SmartServo;

	//private FRIConfiguration _friConfiguration = null;
	//private FRISession       _friSession = null;
	//private FRIJointOverlay  _motionOverlay = null;
	private FRIMode _FRIModeRunnable = null;
	private Thread _FRIModeThread = null;
    boolean waitingForUserToEndFRIMode = false;

	private SmartServo         _smartServoMotion = null;
	private ISmartServoRuntime _smartServoRuntime = null;

	private grl.flatbuffer.KUKAiiwaState _currentKUKAiiwaState = null;
	private grl.flatbuffer.KUKAiiwaState _previousKUKAiiwaState = null;
	private AbstractMotionControlMode _activeMotionControlMode;
	private AbstractMotionControlMode _smartServoMotionControlMode;
	private UpdateConfiguration _updateConfiguration;
	private IRecovery _pausedApplicationRecovery = null;
	private PhysicalObject _toolAttachedToLBR;
	private HandGuidingMotion _handGuidingMotion;
	/**
	 *  gripper, tool or other physically attached object
	 *  see "Template Data" panel in top right pane
	 *  of Sunrise Workbench. This can't be created
	 *  at runtime so we create one for you.
	 */
	private Tool    _flangeAttachment;
	private JointLimits _jointLimits;
	private double[] _maxAllowedJointLimits;
	private double[] _minAllowedJointLimits;
	private JointImpedanceControlMode _teachControlMode;
	private TeachMode _teachModeRunnable = null;
	private Thread _teachModeThread = null;
    boolean waitingForUserToEndTeachMode = false;

//	private MediaFlangeIOGroup _mediaFlangeIOGroup; // this is an end effector flange with a blue ring and buttons on the arm tip

    static final boolean _flexFellowPresent = false; // this is a white an orange robot cart you can attach your arm to
	static final boolean _mediaFlangeIOPresent = false;

	private boolean _canServo = true;
	// when we receive a message
	int message_counter = 0;
	int message_counter_since_last_mode_change = 0;
	
	private Lock configureSmartServoLock = new ReentrantLock();

//    private FlexFellowIOGroup _flexFellowIOGroup;

	@Override
	public void initialize()
	{
		_startStopUI = new StartStopSwitchUI(this);
		_processDataManager = new ProcessDataManager(this);
		_lbrController = (Controller) getContext().getControllers().toArray()[0];
		_lbr = (LBR) _lbrController.getDevices().toArray()[0];

//		if(_flexFellowPresent) _flexFellowIOGroup = new FlexFellowIOGroup(_lbrController);
		if(_mediaFlangeIOPresent) {
//			_mediaFlangeIOGroup = new MediaFlangeIOGroup(_lbrController);
//			_mediaFlangeIOGroup.setLEDBlue(true);
		}

        //_friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _processDataManager.get_FRI_KONI_LaptopIPAddress());
        //_friConfiguration.setSendPeriodMilliSec(4);

        //_friSession = new FRISession(_friConfiguration);

		_flangeAttachment = getApplicationData().createFromTemplate("FlangeAttachment");
		_pausedApplicationRecovery = getRecovery();

		LoadData _loadData = new LoadData();
		_loadData.setMass(_processDataManager.getEndEffectorWeight());
		_loadData.setCenterOfMass(_processDataManager.getEndEffectorX(),
				_processDataManager.getEndEffectorY(),
				_processDataManager.getEndEffectorZ());
		//_toolAttachedToLBR = new Tool("Tool", _loadData);
		// TODO: make configurable via C++ API?
		_toolAttachedToLBR = _flangeAttachment;
		_toolAttachedToLBR.attachTo(_lbr.getFlange());

		_jointLimits = _lbr.getJointLimits();

		// used when setting limits in _HandGuidingMotion
		_maxAllowedJointLimits = _jointLimits.getMaxJointPosition().get();
		_minAllowedJointLimits = _jointLimits.getMinJointPosition().get();
		for (int i = 0; i < _lbr.getJointCount(); i++) {
			_maxAllowedJointLimits[i] -= 0.05;
			_minAllowedJointLimits[i] += 0.05;
		}


		_teachControlMode = new JointImpedanceControlMode(_lbr.getJointCount())
								.setStiffnessForAllJoints(0.1)
								.setDampingForAllJoints(0.7);


		_teachModeRunnable = new TeachMode(
				_flangeAttachment,
				_maxAllowedJointLimits,
				_minAllowedJointLimits);
		_teachModeRunnable.setLogger(getLogger());
		_teachModeThread = new Thread(_teachModeRunnable);
		_teachModeThread.start();


        int millisecondsPerFRITimeStep = 4;
		_FRIModeRunnable = new FRIMode(
				_lbr,
				_processDataManager.get_FRI_KONI_LaptopIPAddress(), millisecondsPerFRITimeStep);
		_FRIModeRunnable.setLogger(getLogger());
		_FRIModeThread = new Thread(_FRIModeRunnable);
		_FRIModeThread.start();
	}

	@Override
	public void run()
	{

		getLogger().info("GRL_Driver from github.com/ahundt/grl starting...\nUDP Connecting to: " + _processDataManager.get_ZMQ_MASTER_URI());

		UDPManager udpMan = new UDPManager(_processDataManager.get_controllingLaptopIPAddress(), _processDataManager.get_controllingLaptopJAVAPort() ,getLogger());

		// if stop is ever set to true the program stops running and exits
		boolean stop;
		stop = udpMan.connect();

		IMotionContainer currentMotion = null;

		boolean newConfig = false;

		// TODO: Let user set mode (teach/joint control from tablet as a backup!)
		//this.getApplicationData().getProcessData("DefaultMode").



		// TODO: add a message that we send to the driver with data log strings
		while (!stop && !_startStopUI.is_stopped() && _lbr.getSafetyState().getEmergencyStopInt()==EmergencyStop.INACTIVE) {
			message_counter+=1;
			_currentKUKAiiwaState = udpMan.waitForNextMessage();
			
			//Some bug in this, just set _previousKUKAiiwaState = _currentKUKAiiwaState before looping/continue?
			_previousKUKAiiwaState = udpMan.getPrevMessage();  


			//////////////////////////////////////////
			// Process Arm Configuration staring here, such as Changing from FRI to SmartServo based commanding
			//////////////////////////////////////////
			/// TODO: Get any configuration updates here and apply them
			if( _currentKUKAiiwaState != null && _currentKUKAiiwaState.armConfiguration() != null
					//&& _currentKUKAiiwaState.armConfiguration().stateType() != _previousKUKAiiwaState.armControlState().stateType())
			  )
			{
				/// TODO: Only change configuration when it changes, not every time
				_commandInterface = _currentKUKAiiwaState.armConfiguration().commandInterface();
				_monitorInterface = _currentKUKAiiwaState.armConfiguration().monitorInterface();

			}

			//////////////////////////////////////////
			// Process Arm Control Commands staring here, such as new joint angles or switching to teach mode
			//////////////////////////////////////////

			// Print out a notice when switching modes
			if( message_counter == 1 || (_previousKUKAiiwaState != null && _previousKUKAiiwaState.armControlState() != null &&
					_currentKUKAiiwaState.armControlState().stateType() != _previousKUKAiiwaState.armControlState().stateType()))
			{
				getLogger()
				.info("Switching mode: "
						+ ArmState.name(_currentKUKAiiwaState.armControlState().stateType()));
				message_counter_since_last_mode_change  = 0;
			} else {

				message_counter_since_last_mode_change++;
			}

			if(_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.ShutdownArm){
				///////////////////////////////////////////////
				// ShutdownArm
				getLogger()
				.info("ShutdownArm command received, stopping GRL_Driver...");
				stop = true;
			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.TeachArm) {
				///////////////////////////////////////////////
				///////////////////////////////////////////////
				// TeachArm teach mode
				///////////////////////////////////////////////
				///////////////////////////////////////////////

				if(message_counter!=1 && (_previousKUKAiiwaState == null || _previousKUKAiiwaState.armControlState()==null)) {
					continue;
				}
				else if(message_counter==1
						|| _currentKUKAiiwaState.armControlState().stateType()!=_previousKUKAiiwaState.armControlState().stateType()
						|| (currentMotion != null && currentMotion.isFinished()) ) {
					// Teach mode, changing from some other mode

					if (currentMotion != null) {
						if(!currentMotion.isFinished()) {
							currentMotion.cancel();
						} else {
							getLogger().info("Hand Guiding Motion has finished!");
						}
					}

                    if(!cancelSmartServo()) continue;

					getLogger().warn("Enabling Teach Mode with gravity compensation. mode id code = " +
							_currentKUKAiiwaState.armControlState().stateType());

//                  // comment/uncomment depending on if you have a flexfellow with lights
//					if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightYellow(true);
//					if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightRed(false);

                    // useHandGuidingMotion==true activates the KUKA supplied HandGuidingMotion
					// Please note that the KUKA supplied HandGuidingMotion requires configuration
					// of a button that lets you exit HandGuidingMotion mode. Some KUKA machines
					// come with the button on the end effector flange, but others require you to
					// attach it to the controller box.
					//
					// useHandGuidingMotion==true activates a simple gravity copensation
					// mode that lets you do something similar,
					// though it will halt when joint limtis are reached.
					//
					// TODO: Move to an enum of possible handguidingmotion modes and make configurable from either the pendant or C++ interface
					boolean useHandGuidingMotion = false;

					if(useHandGuidingMotion)
					{

						_teachModeRunnable.enable();

						getLogger().warn("Started teach thread!");
					}
					else
					{
						currentMotion = _flangeAttachment.moveAsync(positionHold(_teachControlMode, -1, TimeUnit.SECONDS));
					}
				}

			}
			else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.MoveArmTrajectory) {
				///////////////////////////////////////////////
				// MoveArmTrajectory mode (sequence of joint angles)
				///////////////////////////////////////////////
                getLogger().error("MoveArmTrajectory Not Yet Implemented!");

				// create an JointPosition Instance, to play with
				JointPosition destination = new JointPosition(
						_lbr.getJointCount());
				// TODO: not fully implemented
				if(!cancelSmartServo()) continue;
				if(!cancelTeachMode()) continue;

				if (currentMotion != null) currentMotion.cancel();



				MoveArmTrajectory mat;
				if(_currentKUKAiiwaState.armControlState() != null)
				{
					mat = (MoveArmTrajectory)_currentKUKAiiwaState.armControlState().state(new MoveArmTrajectory());
				} else {
					getLogger().error("Received null armControlState in servo!");
					continue;
				}

				for (int j = 0; j < mat.trajLength(); j++)
				{

					JointPosition pos = new JointPosition(_lbr.getCurrentJointPosition());

					for (int k = 0; k < destination.getAxisCount(); ++k)
					{
						//destination.set(k, maj.traj(j).position(k));
						pos.set(k, mat.traj(j).position(k));
						currentMotion = _lbr.moveAsync(ptp(pos));
					}

				}
			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.MoveArmJointServo) {
				///////////////////////////////////////////////
				///////////////////////////////////////////////
				// MoveArmJointServo mode
				///////////////////////////////////////////////
				///////////////////////////////////////////////

				if (currentMotion != null && _commandInterface!=grl.flatbuffer.KUKAiiwaInterface.FRI) currentMotion.cancel();
				if(!cancelTeachMode()) continue;

		        // TODO: switch control mode in config change check? or check in individual armControlState?
		        // Print out a notice when switching controlMode
		        if(_previousKUKAiiwaState != null)
		        {
		        	try{
			          if(_smartServoMotion == null){
				        	// Initialize Smart servo the first time
				        	getLogger().info("Initializing Smart Servo in " 
				        			+ grl.flatbuffer.EControlMode.name(_currentKUKAiiwaState.armConfiguration().controlMode()));
				        	switchSmartServoMotion(_currentKUKAiiwaState.armConfiguration().controlMode());
				        	continue;
			          }else if(_currentKUKAiiwaState.setArmConfiguration()){ //If change in mode requested
				           
			        	  //TODO: bug, _previousKUKAiiwaState & _currentKUKAiiwaState are same. 
			        	 /* getLogger().info("Change controlMode requested: "
						              +grl.flatbuffer.EControlMode.name(_previousKUKAiiwaState.armConfiguration().controlMode())
						              +" -> "+grl.flatbuffer.EControlMode.name(_currentKUKAiiwaState.armConfiguration().controlMode()));*/
				           switchSmartServoMotion(_currentKUKAiiwaState.armConfiguration().controlMode());
			          }
		        	}
		        	catch (Exception e)
		        	{
		        		getLogger().error(e.getMessage());	
		        		getLogger().error("Exception in Starting/Switching SmartServo" );
		        		continue;
		        	}
		        }
		        else{
		        	continue;
		        }

				// create an JointPosition Instance, to play with
				JointPosition destination = new JointPosition(_lbr.getJointCount());

				if(_currentKUKAiiwaState.armControlState() == null) {
					getLogger().error("Received null armControlState in servo!");
					continue;
          // return;
				}

				MoveArmJointServo mas = (MoveArmJointServo)_currentKUKAiiwaState.armControlState().state(new MoveArmJointServo());

				if(_commandInterface==grl.flatbuffer.KUKAiiwaInterface.FRI){

					// only start a new motion overlay if there isn't a current one actively commanding
					/// TODO: perhaps it is possible commands won't actually be sent and that will still be valid, modify this if statement to deal with that
					if( !_FRIModeRunnable.isCommandingWaitOrActive())
					{
						getLogger().info("FRI MotionOverlay starting...");
						_FRIModeRunnable.setControlMode(_smartServoMotionControlMode);
						_FRIModeRunnable.enable();
					}

					if(message_counter % 100 == 0) getLogger().info("FRI MotionOverlay Quality Sample: " + _FRIModeRunnable.getQualityString());
				} else if(_smartServoRuntime != null )  {

					grl.flatbuffer.JointState jointState = mas.goal();
					if(jointState.positionLength()!=destination.getAxisCount()){
						if(message_counter % 500 == 0) getLogger().error("Didn't receive correct number of joints! skipping to start of loop...\n");
						continue;
						//return;
					}
					String pos = "pos:";
					for (int k = 0; k < destination.getAxisCount(); ++k)
					{
						double position = jointState.position(k);
						destination.set(k, position);
					    pos = pos + " " + k + ": " + position;
					}


					try {
						if(message_counter % 1000 == 0) getLogger().info("Sample of new Smart Servo Joint destination " + pos);
						_smartServoRuntime.setDestination(destination);
					} catch (CommandInvalidException e) {
						getLogger().error("Could not update smart servo destination! Clearing SmartServo.");
						_smartServoMotion = null;
						_smartServoRuntime = null;
						continue;
					} catch (java.lang.IllegalStateException ex) {
						getLogger().error("Could not update smart servo destination and missed the real exception type! Clearing SmartServo.");
						_smartServoMotion = null;
						_smartServoRuntime = null;
						getLogger().error(ex.getMessage());
						continue;
					}
			   } else {
					getLogger().error("Couldn't issue motion command, smartServo motion was most likely reset. retrying...");
			   }
			   
			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.StopArm) {

				_smartServoRuntime.stopMotion();
				if (currentMotion != null) {
					currentMotion.cancel();
				}
				if(!cancelTeachMode()) continue;

				//tm.setActive(false);
			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.PauseArm) {

				if (currentMotion != null) currentMotion.cancel();
				if(!cancelTeachMode()) continue;
				if(!cancelSmartServo()) continue;

				PositionControlMode controlMode = new PositionControlMode();
				if(message_counter_since_last_mode_change < 2 || message_counter_since_last_mode_change % 100 == 0){
					_lbr.move(positionHold(controlMode,10,TimeUnit.MILLISECONDS));
				}


			} else if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.StartArm) {

				if (currentMotion != null) currentMotion.cancel();
				if(!cancelTeachMode()) continue;
				if(!cancelSmartServo()) continue;
				
				if(message_counter_since_last_mode_change % 500 == 0) getLogger().info("StartArm mode active, connection established!\nHolding Position while waiting for mode change...\n");

				PositionControlMode controlMode = new PositionControlMode();
				if(message_counter_since_last_mode_change < 2 || message_counter_since_last_mode_change % 100 == 0){
					_lbr.move(positionHold(controlMode,10,TimeUnit.MILLISECONDS));
				}

			} else {
				System.out.println("Unsupported Mode! stopping");
				stop = true;
			}
			

            ///////////////////////////////////////////////////////////////////////////
 			/// Sending commands back to the C++ interface here
			/// Reading sensor values from Java Interface and sending them thrugh UDP
			if (_currentKUKAiiwaState.armControlState().stateType() == grl.flatbuffer.ArmState.MoveArmJointServo){


				/// Note: Be aware that calls like this get data from the realtime system
				///       and are very slow, adding about 2ms each.
				/// @todo TODO(ahundt) Move to another thread, this may help avoid the slowdown detailed above
				Vector force = _lbr.getExternalForceTorque(_lbr.getFlange()).getForce();

				double force_x = force.getX();
				double force_y = force.getY();
				double force_z = force.getZ();
				double torque_x = 0;
				double torque_y = 0;
				double torque_z = 0;

				FlatBufferBuilder builder = new FlatBufferBuilder(0);

				int fb_wrench = Wrench.createWrench(builder, force_x, force_y, force_z, torque_x, torque_y, torque_z, 0, 0, 0);

				KUKAiiwaMonitorState.startKUKAiiwaMonitorState(builder);
				KUKAiiwaMonitorState.addCartesianWrench(builder, fb_wrench);
				int monitorStateOffset = KUKAiiwaMonitorState.endKUKAiiwaMonitorState(builder);

				KUKAiiwaState.startKUKAiiwaState(builder);
				KUKAiiwaState.addMonitorState(builder, monitorStateOffset);
				int[] statesOffset = new int[1];
				statesOffset[0] = KUKAiiwaState.endKUKAiiwaState(builder);

				int statesVector = KUKAiiwaStates.createStatesVector(builder, statesOffset);

				KUKAiiwaStates.startKUKAiiwaStates(builder);
				KUKAiiwaStates.addStates(builder, statesVector);
				int KUKAiiwaStatesOffset = KUKAiiwaStates.endKUKAiiwaStates(builder);


				builder.finish(KUKAiiwaStatesOffset);
				byte[] msg = builder.sizedByteArray();

			try {
				udpMan.sendMessage(msg, msg.length);
			} catch (IOException e) {
				// failed to send message in GRL_Driver.java
			}

			}
			
			{_previousKUKAiiwaState = _currentKUKAiiwaState;}

		} // end primary while loop


		// done
		udpMan.stop();
		_teachModeRunnable.stop();
		if (_updateConfiguration!=null && _updateConfiguration.get_FRISession() != null) {
			_updateConfiguration.get_FRISession().close();
		}

		getLogger()
		.info("UDP connection closed.\nExiting...\nThanks for using the\nGRL_Driver from github.com/ahundt/grl\nGoodbye!");
		//System.exit(1);
	}

	/**
	 *
	 * @return true if teach mode completed successfully; false if we are waiting on a user to press a physical button
	 */
	private boolean cancelTeachMode() {
		if (_teachModeThread != null) {
			//getLogger().warn("Cancelling teach mode...");
			_canServo = _teachModeRunnable.cancel();
//			if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightYellow(false);

			if(!_teachModeRunnable.isEnableEnded() &&
				_currentKUKAiiwaState.armControlState().stateType() != grl.flatbuffer.ArmState.TeachArm){

				if(message_counter % 30 == 0) getLogger().warn("Can't Exit Teach Mode Yet,\n Did You Press The Hand Guiding Button?");
                waitingForUserToEndTeachMode = true;
//				if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightRed(true);
				return false;
			}
            else if(_teachModeRunnable.isEnableEnded()
                    && _currentKUKAiiwaState.armControlState().stateType() != grl.flatbuffer.ArmState.TeachArm
                    && waitingForUserToEndTeachMode)
            {
					PositionControlMode controlMode = new PositionControlMode();
					_lbr.move(positionHold(controlMode,10,TimeUnit.MILLISECONDS));
                    waitingForUserToEndTeachMode = false;
//        			if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightRed(false);
            }
		}
		return true;
	}



	/**
	 *
	 * @return true if FRI mode completed successfully; false if we are waiting on a user to press a physical button
	 */
	private boolean cancelFRIMode() {
		if (_FRIModeThread != null) {
			//getLogger().warn("Cancelling teach mode...");
			_canServo = _FRIModeRunnable.cancel();
//			if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightYellow(false);

			if(!_FRIModeRunnable.isEnableEnded() &&
				_currentKUKAiiwaState.armControlState().stateType() != grl.flatbuffer.ArmState.MoveArmJointServo){

				if(message_counter % 30 == 0) getLogger().warn("Can't Exit FRI Mode Yet, waiting...");
                waitingForUserToEndFRIMode = true;
//				if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightRed(true);
				return false;
			}
            else if(_FRIModeRunnable.isEnableEnded()
                    && _currentKUKAiiwaState.armControlState().stateType() != grl.flatbuffer.ArmState.MoveArmJointServo
                    && waitingForUserToEndFRIMode)
            {
					PositionControlMode controlMode = new PositionControlMode();
					_lbr.move(positionHold(controlMode,10,TimeUnit.MILLISECONDS));
                    waitingForUserToEndFRIMode = false;
//        			if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightRed(false);
            }
		}
		return true;
	}

	/**
	 *
	 * @return true if teach mode completed successfully; false if something went wrong
	 */
    private boolean cancelSmartServo(){
		if (_smartServoRuntime != null) {
			_smartServoRuntime.stopMotion();
			_smartServoMotion = null;
			_smartServoRuntime = null;
//			if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightGreen(false);
			getLogger().info("Ending smart servo!");
		}
		cancelFRIMode();
        return true;
    }

	/**
	 * Checks if a SmartServoMode is of the same type as a MotionControlMode from KUKA APIs
	 * @return boolean
	 */
	private boolean isSameControlMode(IMotionControlMode kukacm, byte controlMode) {
		if (kukacm == null) return false;
		String roscmname = null;
		switch (controlMode) {
		case grl.flatbuffer.EControlMode.POSITION_CONTROL_MODE:
			roscmname = "PositionControlMode";
			break;
		case grl.flatbuffer.EControlMode.JOINT_IMP_CONTROL_MODE:
			roscmname = "JointImpedanceControlMode";
			break;
		case grl.flatbuffer.EControlMode.CART_IMP_CONTROL_MODE:
			roscmname = "CartesianImpedanceControlMode";
			break;
		default:
			roscmname = null;
			break;
		/*// TODO: add these to EControlMode
		case grl.flatbuffer.EControlMode.DESIRED_FORCE:
			roscmname = "CartesianSineImpedanceControlMode";
			break;
		case grl.flatbuffer.EControlMode.SINE_PATTERN:
			roscmname = "CartesianSineImpedanceControlMode";
			break;
		*/
		}
		String kukacmname = kukacm.getClass().getSimpleName();

		return roscmname.equals(kukacmname);
	}
	
	/**
	 * Initialize the appropriate control mode based on passed parameters
	 *
	 * @param controlMode grl.flatbuffer.EControlMode
	 * @return
	 */
	public AbstractMotionControlMode getMotionControlMode(byte controlMode, boolean defaultParams)
	{
		AbstractMotionControlMode mcm = null;

		if(controlMode == grl.flatbuffer.EControlMode.POSITION_CONTROL_MODE){
			mcm = new PositionControlMode();
		} else if(controlMode==grl.flatbuffer.EControlMode.CART_IMP_CONTROL_MODE){
			
			if(defaultParams){
				// TODO: Get missing default params from processData
				/// @note setMaxCartesianVelocity STOPS THE ROBOT ABOVE THAT VELOCITY RATHER THAN CAPPING THE VELOCITY
				CartesianImpedanceControlMode cicm = new CartesianImpedanceControlMode();
				cicm.setMaxCartesianVelocity(1000, 1000, 1000, 6.3, 6.3, 6.3);
				cicm.setMaxPathDeviation(1000, 1000, 1000, 5, 5, 5);
				cicm.setNullSpaceDamping(0.5);
				cicm.setNullSpaceStiffness(2);
				cicm.setMaxControlForce(200, 200, 200, 200, 200, 200, true);
		
				cicm.parametrize(CartDOF.X).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessX());
				cicm.parametrize(CartDOF.Y).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessY());
				cicm.parametrize(CartDOF.Z).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessZ());
				cicm.parametrize(CartDOF.A).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessA());
				cicm.parametrize(CartDOF.B).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessB());
				cicm.parametrize(CartDOF.C).setStiffness(_processDataManager.get_CartesianImpedenceStiffnessC());
				
				cicm.parametrize(CartDOF.X).setDamping(_processDataManager.get_CartesianImpedenceDampingX());
				cicm.parametrize(CartDOF.Y).setDamping(_processDataManager.get_CartesianImpedenceDampingY());
				cicm.parametrize(CartDOF.Z).setDamping(_processDataManager.get_CartesianImpedenceDampingZ());
				cicm.parametrize(CartDOF.A).setDamping(_processDataManager.get_CartesianImpedenceDampingA());
				cicm.parametrize(CartDOF.B).setDamping(_processDataManager.get_CartesianImpedenceDampingB());
				cicm.parametrize(CartDOF.C).setDamping(_processDataManager.get_CartesianImpedenceDampingC());
				mcm = cicm;
			}
			else{
				
				CartesianImpedanceControlMode cicm = new CartesianImpedanceControlMode();
				
		    	cicm.setMaxCartesianVelocity(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().x(),
						        _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().y(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().position().z(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r3(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r2(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxCartesianVelocity().rotation().r1());
				cicm.setMaxPathDeviation(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().x(),
							      _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().y(),
							      _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().position().z(),
							      _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r3(),
							      _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r2(),
							      _currentKUKAiiwaState.armConfiguration().setCartImpedance().maxPathDeviation().rotation().r1());
		    	cicm.setNullSpaceDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().nullspaceDamping());
		    	cicm.setNullSpaceStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().nullspaceStiffness());
		    	cicm.setMaxControlForce(_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().x(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().y(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().position().z(),
			 							_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r3(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r2(),
										_currentKUKAiiwaState.armConfiguration().setCartImpedance().maxControlForce().rotation().r1(), true);
		
		      cicm.parametrize(CartDOF.X).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().x());
		      cicm.parametrize(CartDOF.Y).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().y());
		      cicm.parametrize(CartDOF.Z).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().position().z());
		      cicm.parametrize(CartDOF.A).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r3());
		      cicm.parametrize(CartDOF.B).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r2());
		      cicm.parametrize(CartDOF.C).setStiffness(_currentKUKAiiwaState.armConfiguration().setCartImpedance().stiffness().rotation().r1());
		
		      cicm.parametrize(CartDOF.X).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().x());
		      cicm.parametrize(CartDOF.Y).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().y());
		      cicm.parametrize(CartDOF.Z).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().position().z());
		      cicm.parametrize(CartDOF.A).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r3());
		      cicm.parametrize(CartDOF.B).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r2());
		      cicm.parametrize(CartDOF.C).setDamping(_currentKUKAiiwaState.armConfiguration().setCartImpedance().damping().rotation().r1());
		      mcm = cicm;
			}
		} else if(controlMode==grl.flatbuffer.EControlMode.JOINT_IMP_CONTROL_MODE){

			if(defaultParams){
				JointImpedanceControlMode jicm =  new JointImpedanceControlMode(_lbr.getJointCount())
										.setStiffnessForAllJoints(_processDataManager.get_JointImpedenceStiffness())
										.setDampingForAllJoints(_processDataManager.get_JointImpedenceDamping());
				// TODO: read relevant stiffness/damping params
				//cicm.parametrize(CartDOF.X).setStiffness(stiffnessX);
				//cicm.parametrize(CartDOF.Y).setStiffness(stiffnessY);
				//cicm.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
	
				mcm = jicm;
			}
			else{
				JointImpedanceControlMode jicm =  new JointImpedanceControlMode(_lbr.getJointCount());
				
				if(_lbr.getJointCount() != _currentKUKAiiwaState.armConfiguration().setJointImpedance().stiffnessLength())
				{
					getLogger().info("stiffness/damping vector size is not correct. Set default values for now");
					//the length of stiffness/damping vector is not correct. Set default values for now
					jicm.setStiffnessForAllJoints(_processDataManager.get_JointImpedenceStiffness());
					jicm.setDampingForAllJoints(_processDataManager.get_JointImpedenceDamping());
				}
				else{
					double[] stiffness = new double[_lbr.getJointCount()];
					double[] damping = new double[_lbr.getJointCount()];
					for(int k = 0; k < _lbr.getJointCount(); ++k){
						stiffness[k] = _currentKUKAiiwaState.armConfiguration().setJointImpedance().stiffness(k);
						damping[k] = _currentKUKAiiwaState.armConfiguration().setJointImpedance().damping(k);
					}
					jicm.setStiffness(stiffness);
					jicm.setDamping(damping);	
				}
				mcm = jicm;
			}
		}
		return mcm;
	}

	boolean updateConfig(grl.flatbuffer.KUKAiiwaArmConfiguration newConfig){

		if(newConfig == null) return false;

		if(_currentKUKAiiwaState.armConfiguration().controlMode()!=_previousKUKAiiwaState.armConfiguration().controlMode())
		{

			_smartServoRuntime.changeControlModeSettings(_activeMotionControlMode);
		}

		_commandInterface = _currentKUKAiiwaState.armConfiguration().controlMode();

		return true;
	}

  /**
   * Allows to switch control mode on the fly. Kills a smartServo motion and creates a new one with the given controlMode.
   * @param controlMode
   */
  private void switchSmartServoMotion(byte controlMode) {

	boolean defaultParams = false;
    configureSmartServoLock.lock();

	if(_smartServoMotion!=null){
		if(isSameControlMode(_smartServoMotion.getMode(), controlMode)) { // We can just change the parameters if the control strategy is the same.
			if (!(_smartServoMotion.getMode() instanceof PositionControlMode)) { // We are in PositioControlMode and the request was for the same mode, there are no parameters to change.
				getLogger().info("Changing parameters of Smart Servo in " 
	        			+ grl.flatbuffer.EControlMode.name(controlMode));
				_smartServoMotion.getRuntime().changeControlModeSettings(getMotionControlMode(controlMode, defaultParams));
				configureSmartServoLock.unlock();
				return;
			}
		} 
	    getLogger().info("switching controlMode requested: "
	            + _smartServoMotion.getMode().getClass().getSimpleName()
	            +" -> "+grl.flatbuffer.EControlMode.name(controlMode));
	}
	else
		getLogger().info("Setting up Smart Servo " + grl.flatbuffer.EControlMode.name(controlMode));

    SmartServo oldmotion = _smartServoMotion;

    JointPosition destination = new JointPosition(_lbr.getCurrentJointPosition());
    _smartServoMotion = new SmartServo(destination);

    _smartServoMotionControlMode = getMotionControlMode(controlMode, defaultParams);

    /*
     *
     * Note: The Validation itself justifies, that in this very time
     * instance, the load parameter setting was sufficient. This does not
     * mean by far, that the parameter setting is valid in the sequel or
     * lifetime of this program
     */
    try
    { // TODO: only do this if EControlMode != POSITION_CONTROL_MODE ?
      // TODO: set _smartServoMotionControlMode to POSITION if exception?
      if (!ServoMotion.validateForImpedanceMode(_toolAttachedToLBR))
      {
        getLogger().info("Validation of torque model failed - correct your mass property settings");
        getLogger().info("SmartServo will be available for position controlled mode only, until validation is performed");
      }

    }
    catch (IllegalStateException e)
    {
      getLogger().info("Omitting validation failure for this sample\n"
          + e.getMessage());
    }

    _smartServoMotion.setMode(_smartServoMotionControlMode);

    if (oldmotion != null) {
      oldmotion.getRuntime().stopMotion();
    }

    // Set the motion properties to % of system abilities. For example .2 is 20% of systems abilities
    // TODO: load these over C++ interface
    _smartServoMotion
      .setSpeedTimeoutAfterGoalReach(0.1)
      .setJointAccelerationRel(_processDataManager.get_jointAccelRel())
      .setJointVelocityRel(_processDataManager.get_jointVelRel())
      .setMinimumTrajectoryExecutionTime(20e-3);

    // turn on a physical green light bulb attached to the robot base
//  if(_flexFellowPresent) _flexFellowIOGroup.setSignalLightGreen(true);

    _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(_smartServoMotion);

    getLogger().info("Setting up Smart Servo runtime");
    try {
      // TODO: make motion control mode configurable over UDP interface
      _smartServoRuntime = _smartServoMotion.getRuntime();
      _smartServoRuntime.activateVelocityPlanning(true);
      //_smartServoRuntime.changeControlModeSettings(_smartServoMotionControlMode);
      _activeMotionControlMode = _smartServoMotionControlMode;
    } catch (java.lang.IllegalStateException ex) {
      getLogger().error("Could not retrieve SmartServo runtime!");
      _smartServoRuntime = null;
      _smartServoMotion = null;
      getLogger().error(ex.getMessage());
      configureSmartServoLock.unlock();
      return;
    }
    
    configureSmartServoLock.unlock();
  }


	/**
	 * main.
	 *
	 * @param args
	 *            args
	 */
	public static void main(final String[] args)
	{
		final GRL_Driver app = new GRL_Driver();
		app.runApplication();
	}

}
