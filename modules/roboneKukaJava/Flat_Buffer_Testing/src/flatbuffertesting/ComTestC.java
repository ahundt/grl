package flatbuffertesting;

import java.nio.ByteBuffer;
 
import org.jeromq.ZMQ;
import org.jeromq.ZMQ.Context;
import org.jeromq.ZMQ.Socket;
 
public class ComTestC {
    static final int NUM_MSG = 10;
    static final String ADDRESS = "tcp://127.0.0.1:9998";
    public static void main(String[] args) throws Exception {
 
        Context context = ZMQ.context(1);
        Socket subscriber = context.socket(ZMQ.DEALER);
 
        subscriber.connect("tcp://127.0.0.1:9998");
        subscriber.setRcvHWM(1000000);
        System.out.println("Connected to: "+ADDRESS);
        
        int c = 0;
    
        while (true) {
            byte[] data = subscriber.recv();
            ByteBuffer bb = ByteBuffer.wrap(data);
            VrepControlPoint controlpoint = VrepControlPoint.getRootAsVrepControlPoint(bb);
            Vector3d position = controlpoint.position();
            System.out.println("Position "+c+":");
            System.out.println(position);
            if (c >= NUM_MSG) {
                    break;
            }
        }
        subscriber.close();
        Thread.sleep(1000);
        context.term();
        System.exit(1);
    }
}