package grl;

import java.nio.ByteBuffer;

import com.kuka.task.ITaskLogger;
import java.io.*;
import java.net.*;

public class UDPManager {
	
	DatagramSocket socket = null;

	ITaskLogger logger;
	String _Remote_IP;
	int _Remote_Port;
	
	InetAddress _address_send = null;
	
	int statesLength = 0;
	long message_counter = 0;
	long noMessageCounter = 0;
	long noMessageCounterLimit = 9999999;
	private grl.flatbuffer.KUKAiiwaStates _currentKUKAiiwaStates = null;
	private grl.flatbuffer.KUKAiiwaState _currentKUKAiiwaState = null;
	private grl.flatbuffer.KUKAiiwaState _previousKUKAiiwaState = null;

	byte[] recBuf = new byte[1400];
	ByteBuffer bb = null;
	boolean stop;

	long startTime;
	long elapsedTime;
	long lastMessageStartTime;
	long lastMessageElapsedTime;
	long lastMessageTimeoutMilliseconds = 1000;
	
	int retriesAllowed = 3;
	int retriesAttempted = 0;
	
	public UDPManager(String laptopIP, String laptopPort, ITaskLogger errorlogger) {
		super();
		this.logger = errorlogger;
		_Remote_IP = laptopIP;
		_Remote_Port = Integer.parseInt(laptopPort);
		
		try {
			_address_send = InetAddress.getByName(_Remote_IP);
		} catch (UnknownHostException e) {
			logger.error("Could not create InetAddress for sending");
		}
		
	}

	/**
	 * Blocks until a connection is established or stop() is called.
	 * 
	 * @return error code: false on success, otherwise failure (or told to stop)
	 * @throws UnknownHostException 
	 */
	public boolean connect() {

		logger.info("Waiting for connection initialization...");
			
		try {
			socket = new DatagramSocket();
		} catch (SocketException e1) {
			logger.info("failed to create socket.");
		}
		

		    
		try {
			socket.setSoTimeout(100);
		} catch (SocketException e1) {
			logger.error("UDPManager failed to set socket timeout");
		}
		
		startTime = System.currentTimeMillis();
		elapsedTime = 0L;
		grl.flatbuffer.KUKAiiwaStates newKUKAiiwaStates = null;
		int newStatesLength = 0; 
		
		boolean connectionEstablished = false;
		
		while(newStatesLength<1 && newKUKAiiwaStates == null){
			
			DatagramPacket packet = new DatagramPacket(recBuf, recBuf.length);
			
			// continues sending dummy messages until the server receives the address of this machine and sends a message back
			while (!connectionEstablished){
				connectionEstablished = preConnect();
				if (stop) {
					logger.info("Stopping program.");
					return true; // asked to exit
				}
			}
			
			if(packet.getLength() > 0){

				bb = ByteBuffer.wrap(recBuf);
				
				newKUKAiiwaStates = grl.flatbuffer.KUKAiiwaStates.getRootAsKUKAiiwaStates(bb);
				newStatesLength = newKUKAiiwaStates.statesLength();
		
			}

			if (stop) {
				logger.info("Stopping program.");
				return true; // asked to exit
			}
		}
		
		_currentKUKAiiwaStates = newKUKAiiwaStates;
		statesLength = newStatesLength;

		logger.info("States initialized...");
		
		startTime = System.currentTimeMillis();
		lastMessageStartTime = startTime;
		lastMessageElapsedTime = System.currentTimeMillis() - lastMessageStartTime;
		elapsedTime = 0L;

		return false; // no error
	}
	
	/// @return true successfully sent and received a message, false otherwise
    private boolean preConnect()
    {
		// Dummy message to send to Remote pc (server), in order for the server to know the address of the client (this machine)
    	/// @todo Should probably just make this a real flatbuffers init message too
		String dummyMessage = "Hi";
		
		DatagramPacket packetSend= new DatagramPacket(dummyMessage.getBytes(), dummyMessage.getBytes().length, _address_send, _Remote_Port);

		try {
			socket.send(packetSend);
		} catch (IOException e1) {
			// Could not send
		}
		try {
			DatagramPacket packet = new DatagramPacket(recBuf, recBuf.length);
			socket.receive(packet);
			return true;
		} catch (SocketTimeoutException e) {
			// TimeOut reached, continue sending until we receive something	
			return false;
		} catch (IOException e) {
			// Could not receive packet
			return false;
		}
    }

	/**
	 * Blocks until a connection is re-established or stop() is called.
	 * 
	 * @return error code: false on success, otherwise failure (or told to stop)
	 * @throws IOException 
	 */

	
	public boolean sendMessage(byte[] msg, int size) throws IOException
	{
		DatagramPacket packet= new DatagramPacket(msg, size, _address_send , _Remote_Port );
		socket.send(packet);
	    return true;
	}
	
	public grl.flatbuffer.KUKAiiwaState waitForNextMessage()
	{
		boolean haveNextMessage = false;
		while(!stop && !haveNextMessage) {
			
			DatagramPacket packet = new DatagramPacket(recBuf, recBuf.length);
			try {
				socket.receive(packet);

				if(packet.getLength() > 0){

					message_counter+=1;
					bb = ByteBuffer.wrap(recBuf);
		
					_currentKUKAiiwaStates = grl.flatbuffer.KUKAiiwaStates.getRootAsKUKAiiwaStates(bb, _currentKUKAiiwaStates);
		
					if(_currentKUKAiiwaStates.statesLength()>0) {
						// initialize the fist state
						grl.flatbuffer.KUKAiiwaState tmp = _currentKUKAiiwaStates.states(0);
						if (tmp == null || tmp.armControlState() == null) {
							noMessageCounter +=1;
							if (message_counter % 100 == 0) {
								logger.warn("NULL ArmControlState message, main UDP FlatBuffer message is arriving but doesn't contain any data/commands!");
							}
							continue;
						} else {
							_previousKUKAiiwaState = _currentKUKAiiwaState;
							_currentKUKAiiwaState = tmp;
						}
							
						if (_currentKUKAiiwaState == null) {
							noMessageCounter+=1;
							logger.error("Missing current state message!");
							continue;
						}
						
						haveNextMessage=true;
						noMessageCounter = 0;
						lastMessageStartTime = System.currentTimeMillis();
					} else {
						logger.error("got a UDP packet but it isn't a valid FlatBuffer message, this is an unexpected state that shouldn't occur. please debug me.");
					}
			//	}
				} else {
					noMessageCounter +=1;
					if (message_counter % 100 == 0) {
						logger.warn("Failed to receive UDP packet from control computer... Trying to re-establish connection");
						preConnect();
					}
				}
			} catch (IOException e) {
				noMessageCounter +=1;
				if (message_counter % 100 == 0) {
					logger.warn("Failed to receive UDP packet from control computer... Trying to re-establish connection");
					preConnect();
				}
			}
			
		}
		
		return _currentKUKAiiwaState;			
	}
	
	public grl.flatbuffer.KUKAiiwaState getCurrentMessage(){
		return _currentKUKAiiwaState;
	}
	
	public grl.flatbuffer.KUKAiiwaState getPrevMessage(){
		return _previousKUKAiiwaState;
	}

	
	
	public boolean isStop() {
		return stop;
	}

	public void setStop(boolean stop) {
		if(stop)
		{
			// done
            socket.close();
            logger.error("socket closed");
			this.stop = stop;
			
		}
	}
	
	public void stop(){
		setStop(true);
	}
	
	protected void finalize() {
		try {
			logger.error("Trying to close socket");
			socket.close();
			logger.error("Socket Closed");
		}
		catch (Exception e)
		{
			logger.error("Could not close socket");
		}
	}

	
	
	
	
	
}
