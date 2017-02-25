// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class KUKAiiwaStates extends Table {
  public static KUKAiiwaStates getRootAsKUKAiiwaStates(ByteBuffer _bb) { return getRootAsKUKAiiwaStates(_bb, new KUKAiiwaStates()); }
  public static KUKAiiwaStates getRootAsKUKAiiwaStates(ByteBuffer _bb, KUKAiiwaStates obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public static boolean KUKAiiwaStatesBufferHasIdentifier(ByteBuffer _bb) { return __has_identifier(_bb, "iiwa"); }
  public KUKAiiwaStates __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public KUKAiiwaState states(int j) { return states(new KUKAiiwaState(), j); }
  public KUKAiiwaState states(KUKAiiwaState obj, int j) { int o = __offset(4); return o != 0 ? obj.__init(__indirect(__vector(o) + j * 4), bb) : null; }
  public int statesLength() { int o = __offset(4); return o != 0 ? __vector_len(o) : 0; }

  public static int createKUKAiiwaStates(FlatBufferBuilder builder,
      int statesOffset) {
    builder.startObject(1);
    KUKAiiwaStates.addStates(builder, statesOffset);
    return KUKAiiwaStates.endKUKAiiwaStates(builder);
  }

  public static void startKUKAiiwaStates(FlatBufferBuilder builder) { builder.startObject(1); }
  public static void addStates(FlatBufferBuilder builder, int statesOffset) { builder.addOffset(0, statesOffset, 0); }
  public static int createStatesVector(FlatBufferBuilder builder, int[] data) { builder.startVector(4, data.length, 4); for (int i = data.length - 1; i >= 0; i--) builder.addOffset(data[i]); return builder.endVector(); }
  public static void startStatesVector(FlatBufferBuilder builder, int numElems) { builder.startVector(4, numElems, 4); }
  public static int endKUKAiiwaStates(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
  public static void finishKUKAiiwaStatesBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset, "iiwa"); }
};

