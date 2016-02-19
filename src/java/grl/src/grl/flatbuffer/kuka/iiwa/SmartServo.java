// automatically generated, do not modify

package grl.flatbuffer.kuka.iiwa;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class SmartServo extends Table {
  public static SmartServo getRootAsSmartServo(ByteBuffer _bb) { return getRootAsSmartServo(_bb, new SmartServo()); }
  public static SmartServo getRootAsSmartServo(ByteBuffer _bb, SmartServo obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public SmartServo __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }


  public static void startSmartServo(FlatBufferBuilder builder) { builder.startObject(0); }
  public static int endSmartServo(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

