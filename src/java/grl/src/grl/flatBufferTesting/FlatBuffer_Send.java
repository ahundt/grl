package grl.flatBufferTesting;

import java.nio.ByteBuffer;
import java.util.Arrays;

import org.jeromq.ZMQ;
import org.jeromq.ZMQ.Context;
import org.jeromq.ZMQ.Socket;

import com.google.flatbuffers.FlatBufferBuilder;
import grl.flatbuffer.VrepControlPoint;
import grl.flatbuffer.Vector3d;
 
public class FlatBuffer_Send {
    static final String ADDRESS = "tcp://127.0.0.1:5563";
    static final int NUM_MSG = 10;
    public static void main(String[] args) throws Exception {
        
        // Prepare our context and publisher
        Context context = ZMQ.context(1);
        Socket publisher = context.socket(ZMQ.DEALER);
        publisher.bind(ADDRESS);
        publisher.setHWM(1000000);
        publisher.setSndHWM(1000000);
        
        // Create a Flat Buffer
        FlatBufferBuilder fbb = new FlatBufferBuilder(1);
        VrepControlPoint.startVrepControlPoint(fbb);
        VrepControlPoint.addPosition(fbb, Vector3d.createVector3d(fbb, 0.0, 1.0, 2.0));
        VrepControlPoint.addRotation(fbb, Vector3d.createVector3d(fbb, 3.0, 4.0, 5.0));
        VrepControlPoint.addRelativeVelocity(fbb, 6.0);
        int cp = VrepControlPoint.endVrepControlPoint(fbb);
        VrepControlPoint.finishVrepControlPointBuffer(fbb, cp);
        
        // Output Flat Buffer for Testing
        byte [] sendByteArray = fbb.sizedByteArray();
        System.out.println(sendByteArray);
        ByteBuffer bb = ByteBuffer.wrap(sendByteArray);
        VrepControlPoint controlpoint = VrepControlPoint.getRootAsVrepControlPoint(bb);
        // Vector3d position = controlpoint.position();
        // EulerXYZd rotation = controlpoint.rotation();
        double [] position = {controlpoint.position().x(), controlpoint.position().y(), controlpoint.position().z()};
        double [] rotation = {controlpoint.rotation().rx(), controlpoint.rotation().ry(), controlpoint.rotation().rz()};
        double vel_rel = controlpoint.relativeVelocity();
        System.out.println(Arrays.toString(position));
        System.out.println(Arrays.toString(rotation));
        System.out.println(vel_rel);
        
        // Send Flat Buffer
        int c = 0;
        while (c < NUM_MSG) {
            c++;
            publisher.send(sendByteArray);
        }
        Thread.sleep(2000);
        publisher.close();
        context.term();
        System.exit(1);
    }
}