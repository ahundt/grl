// automatically generated, do not modify

package grl.flatbuffer.kuka.iiwa;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class CheckTorqueSensor extends Table {
  public static CheckTorqueSensor getRootAsCheckTorqueSensor(ByteBuffer _bb) { return getRootAsCheckTorqueSensor(_bb, new CheckTorqueSensor()); }
  public static CheckTorqueSensor getRootAsCheckTorqueSensor(ByteBuffer _bb, CheckTorqueSensor obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public CheckTorqueSensor __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }


  public static void startCheckTorqueSensor(FlatBufferBuilder builder) { builder.startObject(0); }
  public static int endCheckTorqueSensor(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

