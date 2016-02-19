// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class ShutdownArm extends Table {
  public static ShutdownArm getRootAsShutdownArm(ByteBuffer _bb) { return getRootAsShutdownArm(_bb, new ShutdownArm()); }
  public static ShutdownArm getRootAsShutdownArm(ByteBuffer _bb, ShutdownArm obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public ShutdownArm __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }


  public static void startShutdownArm(FlatBufferBuilder builder) { builder.startObject(0); }
  public static int endShutdownArm(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

