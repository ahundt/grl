package grl;

import java.nio.ByteBuffer;

import org.zeromq.ZMQ;

import com.kuka.task.ITaskLogger;

/**
 *  @brief ZMQManager Handles the ZeroMQ/JeroMQ loading in flatbuffer data.
 * 
 * @author Andrew Hundt
 *
 * @todo support sending data back to the controller PC
 */
public class ZMQManager {
	ZMQ.Context context = null;
	ZMQ.Socket subscriber = null;
	ITaskLogger logger;
	String _ZMQ_MASTER_URI;

	int statesLength = 0;
	long message_counter = 0;
	long noMessageCounter = 0;
	long noMessageCounterLimit = 9999999;
	private grl.flatbuffer.KUKAiiwaStates _currentKUKAiiwaStates = null;
	private grl.flatbuffer.KUKAiiwaState _currentKUKAiiwaState = null;
	private grl.flatbuffer.KUKAiiwaState _previousKUKAiiwaState = null;
	byte [] data = null;
	ByteBuffer bb = null;
	boolean stop;

	long startTime;
	long elapsedTime;
	long lastMessageStartTime;
	long lastMessageElapsedTime;
	long lastMessageTimeoutMilliseconds = 1000;
	
	int retriesAllowed = 3;
	int retriesAttempted = 0;
	
	public ZMQManager(String ZMQ_MASTER_URI, ITaskLogger errorlogger) {
		super();
		this.logger = errorlogger;
		_ZMQ_MASTER_URI = ZMQ_MASTER_URI;
	}

	/**
	 * Blocks until a connection is established or stop() is called.
	 * 
	 * @return error code: false on success, otherwise failure (or told to stop)
	 */
	public boolean connect(){

		logger.info("Waiting for ZMQ connection initialization...");
		this.context = ZMQ.context(1);
		subscriber = context.socket(ZMQ.DEALER);
		subscriber.connect(_ZMQ_MASTER_URI);
		subscriber.setRcvHWM(100000);
		startTime = System.currentTimeMillis();
		elapsedTime = 0L;
		grl.flatbuffer.KUKAiiwaStates newKUKAiiwaStates = null;
		int newStatesLength = 0; 
		
		while(newStatesLength<1 && newKUKAiiwaStates == null){
			if((data = subscriber.recv(ZMQ.DONTWAIT))!=null){
				bb = ByteBuffer.wrap(data);

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

	/**
	 * Blocks until a connection is re-established or stop() is called.
	 * 
	 * @return error code: false on success, otherwise failure (or told to stop)
	 */
	public boolean reconnect(){
		logger.info("Disconnecting...");
		//subscriber.disconnect(_ZMQ_MASTER_URI);
		//subscriber.close();
		//context.close();
		return false;// this.connect();
	}
	
	public grl.flatbuffer.KUKAiiwaState waitForNextMessage()
	{
		boolean haveNextMessage = false;
		while(!stop && !haveNextMessage) {
			
//			elapsedTime = System.currentTimeMillis() - startTime;
//			lastMessageElapsedTime = System.currentTimeMillis() - lastMessageStartTime;
//			
//			if(lastMessageElapsedTime > lastMessageTimeoutMilliseconds)
//			{
//				retriesAttempted++;
//				logger.error("Message rate timeout occurred... ZMQ connection may be dead. Retrying first. \nAttempting to restart connection...\n");	
//				
//				
//				if(retriesAttempted > retriesAllowed){
//					logger.error("Attempting to restart connection...\n");
//					this.reconnect();
//				} else {
//					lastMessageStartTime = System.currentTimeMillis();
//				}
//
//			}
//			
//			else if (noMessageCounter > noMessageCounterLimit)
//			{	
//				logger.error("ZMQ connection seems dead, messages arrive empty.\nAttempting to restart connection...\n");
//				this.reconnect();
//			}
			
			
			if((data = subscriber.recv(ZMQ.DONTWAIT))!=null){
				/// TODO: BUG! noMessageCounter is always set to 0 here and only incremented below, so it will only ever be 0 or 1

				message_counter+=1;
				bb = ByteBuffer.wrap(data);
	
				_currentKUKAiiwaStates = grl.flatbuffer.KUKAiiwaStates.getRootAsKUKAiiwaStates(bb, _currentKUKAiiwaStates);
	
				if(_currentKUKAiiwaStates.statesLength()>0) {
					// initialize the fist state
					grl.flatbuffer.KUKAiiwaState tmp = _currentKUKAiiwaStates.states(0);
					if (tmp == null || tmp.armControlState() == null) {
						noMessageCounter +=1;
						if (message_counter % 100 == 0) {
							logger.warn("NULL ArmControlState message, main ZMQ message is arriving but doesn't contain any data/commands!");
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
					logger.error("got a ZMQ message but it isn't a valid message, this is an unexpected state that shouldn't occur. please debug me.");
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
			subscriber.close();
			context.term();
			this.stop = stop;
			
		}
	}
	
	public void stop(){
		setStop(true);
	}
	
}
