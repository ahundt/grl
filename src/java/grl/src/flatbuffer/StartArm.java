// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class StartArm extends Table {
  public static StartArm getRootAsStartArm(ByteBuffer _bb) { return getRootAsStartArm(_bb, new StartArm()); }
  public static StartArm getRootAsStartArm(ByteBuffer _bb, StartArm obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public StartArm __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }


  public static void startStartArm(FlatBufferBuilder builder) { builder.startObject(0); }
  public static int endStartArm(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

