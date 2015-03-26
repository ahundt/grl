package flatbuffertesting;

import java.util.concurrent.TimeUnit;
 
import org.jeromq.ZMQ;
import org.jeromq.ZMQ.Context;
import org.jeromq.ZMQ.Socket;
 
//https://github.com/miniway/jeromq/tree/master/src/test/java/guide
public class Sub {
    static final int NUM_MSG = 10;
    static final String ADDRESS = "tcp://127.0.0.1:5563";
    public static void main(String[] args) throws Exception {
        long bTime = System.currentTimeMillis();
 
        // Prepare our context and dealer
        Context context = ZMQ.context(1);
        Socket subscriber = context.socket(ZMQ.DEALER);
 
        subscriber.connect(ADDRESS);
        subscriber.setRcvHWM(1000000);
        System.out.println("Connected to:  "+ADDRESS);
        
        int c = 0;
        while (true) {
            byte [] msg = subscriber.recv();
            c++;
            System.out.println(msg);
            if (c >= NUM_MSG) {
                break;
            }
        }
        
        long secs = TimeUnit.MILLISECONDS.toSeconds(System.currentTimeMillis()- bTime);
        System.out.println("Wuserver done at " + c + " in " + secs + " seconds");
        subscriber.close();
        //Thread.sleep(1000);
        context.term();
        System.exit(1);
    }
}