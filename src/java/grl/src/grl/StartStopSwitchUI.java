package grl;

import java.util.ArrayList;
import java.util.List;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

public class StartStopSwitchUI {

	private RoboticsAPIApplication  _app;
	// configurable toolbars
	private List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
	private List<IUserKey> generalKeys = new ArrayList<IUserKey>();
	private List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();
	
	// gravity compensation stuff
	private IUserKeyBar startstopKeybar;
	private IUserKey startstopKey;
	private IUserKeyListener startstopKeyList;
	private boolean startstopEnabled = false;
	private boolean startstopSwitched = false;

public StartStopSwitchUI(RoboticsAPIApplication app)
{
	_app = app;


	// gravity compensation - only in ROSMonitor for safety
	startstopKeybar = _app.getApplicationUI().createUserKeyBar("startstop");
	startstopKeyList = new IUserKeyListener() {
		@Override
		public void onKeyEvent(IUserKey key, com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent event) {
			if (event == UserKeyEvent.FirstKeyDown) {
				startstopEnabled = true;
				startstopSwitched = true;
			} else if (event == UserKeyEvent.SecondKeyDown) {
				startstopEnabled = false;
				startstopSwitched = true;
			}
		}
	};
	startstopKey = startstopKeybar.addDoubleUserKey(0, startstopKeyList, true);
	startstopKey.setText(UserKeyAlignment.TopMiddle, "ON");
	startstopKey.setText(UserKeyAlignment.BottomMiddle, "OFF");
	startstopKeybar.publish();
}

public boolean is_stopped()
{
	return startstopEnabled;
}

}
