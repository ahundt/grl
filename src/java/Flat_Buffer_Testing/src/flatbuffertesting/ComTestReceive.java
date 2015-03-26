package flatbuffertesting;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.jeromq.ZMQ;
import org.jeromq.ZMQ.Context;
import org.jeromq.ZMQ.Socket;

 
public class ComTestReceive {
    static final int NUM_MSG = 30;
    static final String ADDRESS = "tcp://10.188.179.31:5563";
    public static void main(String[] args) throws Exception {
 
        // Prepare our context and dealer
        Context context = ZMQ.context(1);
        Socket subscriber = context.socket(ZMQ.DEALER);
 
        subscriber.connect(ADDRESS);
        subscriber.setRcvHWM(1000000);
        System.out.println("Connected to:  "+ADDRESS);
        
        // Receive and Output Flat Buffer
        int c = 0;
        ArrayList<Double> list = new ArrayList<Double>();
        while (true) {
            byte [] data = subscriber.recv();
            ByteBuffer bb = ByteBuffer.wrap(data);
            VrepControlPoint controlpoint = VrepControlPoint.getRootAsVrepControlPoint(bb);
            // double [] position = {controlpoint.position().x(), controlpoint.position().y(), controlpoint.position().z()};
            // double [] rotation = {controlpoint.rotation().rx(), controlpoint.rotation().ry(), controlpoint.rotation().rz()};
            // double vel_rel = controlpoint.relativeVelocity();
            // System.out.println(Arrays.toString(position));
            list.add(controlpoint.position().x());
            // System.out.println(controlpoint.position().x());
            // System.out.println(Arrays.toString(rotation));
            // System.out.println(vel_rel);
            c++;
            if (c >= NUM_MSG) {
                    break;
            }
        }
        System.out.println("Left Loop");
        for (int i=0; i<list.size(); i++){
        	System.out.println(list.get(i));
        }
        subscriber.close();
        Thread.sleep(1000);
        context.term();
        System.exit(1);
    }
}