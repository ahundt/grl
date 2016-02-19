// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class FRI extends Table {
  public static FRI getRootAsFRI(ByteBuffer _bb) { return getRootAsFRI(_bb, new FRI()); }
  public static FRI getRootAsFRI(ByteBuffer _bb, FRI obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public FRI __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public int sendPeriodMillisec() { int o = __offset(4); return o != 0 ? bb.getInt(o + bb_pos) : 0; }
  public int setReceiveMultiplier() { int o = __offset(6); return o != 0 ? bb.getInt(o + bb_pos) : 0; }
  public short portOnRemote() { int o = __offset(8); return o != 0 ? bb.getShort(o + bb_pos) : 0; }
  public short portOnController() { int o = __offset(10); return o != 0 ? bb.getShort(o + bb_pos) : 0; }

  public static int createFRI(FlatBufferBuilder builder,
      int sendPeriodMillisec,
      int setReceiveMultiplier,
      short portOnRemote,
      short portOnController) {
    builder.startObject(4);
    FRI.addSetReceiveMultiplier(builder, setReceiveMultiplier);
    FRI.addSendPeriodMillisec(builder, sendPeriodMillisec);
    FRI.addPortOnController(builder, portOnController);
    FRI.addPortOnRemote(builder, portOnRemote);
    return FRI.endFRI(builder);
  }

  public static void startFRI(FlatBufferBuilder builder) { builder.startObject(4); }
  public static void addSendPeriodMillisec(FlatBufferBuilder builder, int sendPeriodMillisec) { builder.addInt(0, sendPeriodMillisec, 0); }
  public static void addSetReceiveMultiplier(FlatBufferBuilder builder, int setReceiveMultiplier) { builder.addInt(1, setReceiveMultiplier, 0); }
  public static void addPortOnRemote(FlatBufferBuilder builder, short portOnRemote) { builder.addShort(2, portOnRemote, 0); }
  public static void addPortOnController(FlatBufferBuilder builder, short portOnController) { builder.addShort(3, portOnController, 0); }
  public static int endFRI(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

