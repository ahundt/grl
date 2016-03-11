package grl.flatBufferTesting;

import java.nio.ByteBuffer;
 
import org.zeromq.ZMQ;

import grl.flatbuffer.VrepControlPoint;
import grl.flatbuffer.Vector3d;
 
public class FlatBuffer_Receive {
    static final int NUM_MSG = 10;
    static final String ADDRESS = "tcp://127.0.0.1:9998";
    public static void main(String[] args) throws Exception {
 
        ZMQ.Context context = ZMQ.context(1);
        ZMQ.Socket subscriber = context.socket(ZMQ.DEALER);
 
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