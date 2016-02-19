// automatically generated, do not modify

package grl.flatbuffer.kuka.iiwa;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Disabled extends Table {
  public static Disabled getRootAsDisabled(ByteBuffer _bb) { return getRootAsDisabled(_bb, new Disabled()); }
  public static Disabled getRootAsDisabled(ByteBuffer _bb, Disabled obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public Disabled __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }


  public static void startDisabled(FlatBufferBuilder builder) { builder.startObject(0); }
  public static int endDisabled(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

