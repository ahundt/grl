// automatically generated, do not modify

package grl.flatbuffer.kuka.iiwa;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class FRI extends Table {
  public static FRI getRootAsFRI(ByteBuffer _bb) { return getRootAsFRI(_bb, new FRI()); }
  public static FRI getRootAsFRI(ByteBuffer _bb, FRI obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public FRI __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public short sendPeriodMillisec() { int o = __offset(4); return o != 0 ? bb.getShort(o + bb_pos) : 0; }

  public static int createFRI(FlatBufferBuilder builder,
      short sendPeriodMillisec) {
    builder.startObject(1);
    FRI.addSendPeriodMillisec(builder, sendPeriodMillisec);
    return FRI.endFRI(builder);
  }

  public static void startFRI(FlatBufferBuilder builder) { builder.startObject(1); }
  public static void addSendPeriodMillisec(FlatBufferBuilder builder, short sendPeriodMillisec) { builder.addShort(0, sendPeriodMillisec, 0); }
  public static int endFRI(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

