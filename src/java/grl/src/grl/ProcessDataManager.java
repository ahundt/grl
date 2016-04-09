package grl;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

/**
 * Stores variables set in "Process Data" panel on 
 * the kuka iiwa tablet. Loaded from RoboticsAPI.data.xml.
 * @author Andrew Hundt
 *
 */
public class ProcessDataManager {



	private RoboticsAPIApplication  _app;
	private String _controllingLaptopIPAddress;
    private String _controllingLaptopZMQPort;
    private String _controllingLaptop_ROS_MASTER_URI_Port;
    private String _RobotIPAddress;
    private String _FRI_KONI_RobotIPAddress;
    private String _FRI_KONI_LaptopIPAddress;
    private String _ROS_MASTER_URI;
    private String _ZMQ_MASTER_URI;

    private double _jointVelRel;
    private double _jointAccelRel;
    
    // joint impedence
    private double _jointImpedenceDamping;
    private double _jointImpedenceStiffness;
    
    // joint stiffness
    private double _cartesianImpedenceStiffnessX;
    private double _cartesianImpedenceStiffnessY;
    private double _cartesianImpedenceStiffnessZ;
    private double _cartesianImpedenceStiffnessA;
    private double _cartesianImpedenceStiffnessB;
    private double _cartesianImpedenceStiffnessC;
    
    // joint damping
    private double _cartesianImpedenceDampingX;
    private double _cartesianImpedenceDampingY;
    private double _cartesianImpedenceDampingZ;
    private double _cartesianImpedenceDampingA;
    private double _cartesianImpedenceDampingB;
    private double _cartesianImpedenceDampingC;

    // end effector settings
    private double eeWeight;
    private double eeX;
    private double eeY;
    private double eeZ;
    
	public String get_ZMQ_MASTER_URI() {
		return _ZMQ_MASTER_URI;
	}

	public double getEndEffectorWeight() {
		return eeWeight;
	}
	
	public double getEndEffectorX() {
		return eeX;
	}
	
	public double getEndEffectorY() {
		return eeY;
	}
	
	public double getEndEffectorZ() {
		return eeZ;
	}

	private void update_ZMQ_MASTER_URI() {
		this._ZMQ_MASTER_URI = "tcp://" + _controllingLaptopIPAddress + ":" + _controllingLaptopZMQPort;
	}


	public String get_ROS_MASTER_URI() {
		return _ROS_MASTER_URI;
	}

	private void update_ROS_MASTER_URI() {
		this._ROS_MASTER_URI = "http://" + _controllingLaptopIPAddress + ":" + _controllingLaptop_ROS_MASTER_URI_Port;
	}

	public String get_controllingLaptopIPAddress() {
		return _controllingLaptopIPAddress;
	}

	public void set_controllingLaptopIPAddress(String _controllingLaptopIPAddress) {
		this._controllingLaptopIPAddress = _controllingLaptopIPAddress;
		update_ZMQ_MASTER_URI();
		update_ROS_MASTER_URI();
	}

	public String get_controllingLaptopJAVAPort() {
		return _controllingLaptopZMQPort;
	}

	public void set_controllingLaptopJAVAPort(String _controllingLaptopJAVAPort) {
		this._controllingLaptopZMQPort = _controllingLaptopJAVAPort;
		update_ZMQ_MASTER_URI();
	}

	public String get_controllingLaptopROSPort() {
		return _controllingLaptop_ROS_MASTER_URI_Port;
	}

	public void set_controllingLaptopROSPort(String _controllingLaptopROSPort) {
		this._controllingLaptop_ROS_MASTER_URI_Port = _controllingLaptopROSPort;
		update_ROS_MASTER_URI();
	}

	public String get_RobotIPAddress() {
		return _RobotIPAddress;
	}

	public void set_RobotIPAddress(String _RobotIPAddress) {
		this._RobotIPAddress = _RobotIPAddress;
	}

	public String get_FRI_KONI_RobotIPAddress() {
		return _FRI_KONI_RobotIPAddress;
	}

	public void set_FRI_KONI_RobotIPAddress(String _FRI_KONI_RobotIPAddress) {
		this._FRI_KONI_RobotIPAddress = _FRI_KONI_RobotIPAddress;
	}

	public String get_FRI_KONI_LaptopIPAddress() {
		return _FRI_KONI_LaptopIPAddress;
	}

	public void set_FRI_KONI_LaptopIPAddress(String _FRI_KONI_LaptopIPAddress) {
		this._FRI_KONI_LaptopIPAddress = _FRI_KONI_LaptopIPAddress;
	}

	
	public double get_JointImpedenceDamping() {
		return _jointImpedenceDamping;
	}
	
	public double get_JointImpedenceStiffness() {
		return _jointImpedenceStiffness;
	}
	
	public double get_CartesianImpedenceStiffnessX() {
		return _cartesianImpedenceStiffnessX;
	}
	
	public double get_CartesianImpedenceStiffnessY() {
		return _cartesianImpedenceStiffnessY;
	}
	
	public double get_CartesianImpedenceStiffnessZ() {
		return _cartesianImpedenceStiffnessZ;
	}
	
	public double get_CartesianImpedenceStiffnessA() {
		return _cartesianImpedenceStiffnessA;
	}
	
	public double get_CartesianImpedenceStiffnessB() {
		return _cartesianImpedenceStiffnessB;
	}
	
	public double get_CartesianImpedenceStiffnessC() {
		return _cartesianImpedenceStiffnessC;
	}
	

	
	public double get_CartesianImpedenceDampingX() {
		return _cartesianImpedenceDampingX;
	}
	
	public double get_CartesianImpedenceDampingY() {
		return _cartesianImpedenceDampingY;
	}
	
	public double get_CartesianImpedenceDampingZ() {
		return _cartesianImpedenceDampingZ;
	}
	
	public double get_CartesianImpedenceDampingA() {
		return _cartesianImpedenceDampingA;
	}
	
	public double get_CartesianImpedenceDampingB() {
		return _cartesianImpedenceDampingB;
	}
	
	public double get_CartesianImpedenceDampingC() {
		return _cartesianImpedenceDampingC;
	}
    
    public ProcessDataManager(final RoboticsAPIApplication app){
    	this._app = app;
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _controllingLaptopIPAddress = _app.getApplicationData().getProcessData("Laptop_IP").getValue(); //"192.170.10.100";
        _controllingLaptopZMQPort = _app.getApplicationData().getProcessData("Laptop_Port").getValue(); //"192.170.10.100";
        _controllingLaptop_ROS_MASTER_URI_Port = _app.getApplicationData().getProcessData("Laptop_ROS_MASTER_URI_Port").getValue(); //"192.170.10.100";
        //http://172.31.1.100:11311
        
        // **********************************************************************
        // *** change next line to the KUKA address and Port Number           ***
        // **********************************************************************
        _RobotIPAddress = _app.getApplicationData().getProcessData("Robot_IP").getValue(); //"tcp://172.31.1.100:30010";

        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _FRI_KONI_LaptopIPAddress = _app.getApplicationData().getProcessData("Laptop_KONI_FRI_IP").getValue(); //"192.170.10.100";

        
        // **********************************************************************
        // *** change next line to the KUKA address and Port Number           ***
        // **********************************************************************
        _FRI_KONI_RobotIPAddress = _app.getApplicationData().getProcessData("Robot_KONI_FRI_IP").getValue(); //"tcp://172.31.1.100:30010";
        

        _jointVelRel = _app.getApplicationData().getProcessData("JointVelRel").getValue();
        _jointAccelRel = _app.getApplicationData().getProcessData("JointAccelRel").getValue();

        _jointImpedenceDamping = _app.getApplicationData().getProcessData("JointImpedenceDamping").getValue();
        _jointImpedenceStiffness = _app.getApplicationData().getProcessData("JointImpedenceStiffness").getValue();


        _cartesianImpedenceStiffnessX = _app.getApplicationData().getProcessData("CartesianImpedenceStiffnessX").getValue();
        _cartesianImpedenceStiffnessY = _app.getApplicationData().getProcessData("CartesianImpedenceStiffnessY").getValue();
        _cartesianImpedenceStiffnessZ = _app.getApplicationData().getProcessData("CartesianImpedenceStiffnessZ").getValue();
        _cartesianImpedenceStiffnessA = _app.getApplicationData().getProcessData("CartesianImpedenceStiffnessA").getValue();
        _cartesianImpedenceStiffnessB = _app.getApplicationData().getProcessData("CartesianImpedenceStiffnessB").getValue();
        _cartesianImpedenceStiffnessC = _app.getApplicationData().getProcessData("CartesianImpedenceStiffnessC").getValue();

        _cartesianImpedenceDampingX = _app.getApplicationData().getProcessData("CartesianImpedenceDampingX").getValue();
        _cartesianImpedenceDampingY = _app.getApplicationData().getProcessData("CartesianImpedenceDampingY").getValue();
        _cartesianImpedenceDampingZ = _app.getApplicationData().getProcessData("CartesianImpedenceDampingZ").getValue();
        _cartesianImpedenceDampingA = _app.getApplicationData().getProcessData("CartesianImpedenceDampingA").getValue();
        _cartesianImpedenceDampingB = _app.getApplicationData().getProcessData("CartesianImpedenceDampingB").getValue();
        _cartesianImpedenceDampingC = _app.getApplicationData().getProcessData("CartesianImpedenceDampingC").getValue();

        eeWeight = _app.getApplicationData().getProcessData("eeWeight").getValue();
        eeX = _app.getApplicationData().getProcessData("eeX").getValue();
        eeY = _app.getApplicationData().getProcessData("eeY").getValue();
        eeZ = _app.getApplicationData().getProcessData("eeZ").getValue();
        
        update_ROS_MASTER_URI();
        update_ZMQ_MASTER_URI();
    }

	/**
	 * @return the _jointVelRel
	 */
	public double get_jointVelRel() {
		return _jointVelRel;
	}

	/**
	 * @param _jointVelRel the _jointVelRel to set
	 */
	public void set_jointVelRel(double _jointVelRel) {
		this._jointVelRel = _jointVelRel;
	}

	/**
	 * @return the _jointAccelRel
	 */
	public double get_jointAccelRel() {
		return _jointAccelRel;
	}

	/**
	 * @param _jointAccelRel the _jointAccelRel to set
	 */
	public void set_jointAccelRel(double _jointAccelRel) {
		this._jointAccelRel = _jointAccelRel;
	}

}
