package flatbuffertesting;

import java.util.concurrent.TimeUnit;
import org.jeromq.ZMQ;
import org.jeromq.ZMQ.Context;
import org.jeromq.ZMQ.Socket;
 
public class Pub { // Used to be Wuclient
    static final String TOPIC = "topic1";
    static final int NUM_MSG = 1000000;
    public static void main(String[] args) throws Exception {
        long bTime = System.currentTimeMillis();
        // Prepare our context and publisher
        Context context = ZMQ.context(1);
        Socket publisher = context.socket(ZMQ.DEALER);
        publisher.bind("tcp://127.0.0.1:5563");
        publisher.setHWM(1000000);
        publisher.setSndHWM(1000000);
        int c = 0;
        System.out.println("Test");
        while (c < NUM_MSG) {
            publisher.sendMore(TOPIC);
            c++;
            publisher.send("message number "+c);
            System.out.println(c);
        }
        long secs = TimeUnit.MILLISECONDS.toSeconds(System.currentTimeMillis() - bTime);
        System.out.println("Wuclient done at "+c + " in "+secs + " seconds");
        Thread.sleep(2000);
        publisher.close();
        context.term();
        System.exit(1);
    }
}