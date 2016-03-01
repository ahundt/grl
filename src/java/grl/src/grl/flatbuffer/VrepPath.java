// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class VrepPath extends Table {
  public static VrepPath getRootAsVrepPath(ByteBuffer _bb) { return getRootAsVrepPath(_bb, new VrepPath()); }
  public static VrepPath getRootAsVrepPath(ByteBuffer _bb, VrepPath obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public VrepPath __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public VrepControlPoint controlPoints(int j) { return controlPoints(new VrepControlPoint(), j); }
  public VrepControlPoint controlPoints(VrepControlPoint obj, int j) { int o = __offset(4); return o != 0 ? obj.__init(__indirect(__vector(o) + j * 4), bb) : null; }
  public int controlPointsLength() { int o = __offset(4); return o != 0 ? __vector_len(o) : 0; }

  public static int createVrepPath(FlatBufferBuilder builder,
      int controlPoints) {
    builder.startObject(1);
    VrepPath.addControlPoints(builder, controlPoints);
    return VrepPath.endVrepPath(builder);
  }

  public static void startVrepPath(FlatBufferBuilder builder) { builder.startObject(1); }
  public static void addControlPoints(FlatBufferBuilder builder, int controlPointsOffset) { builder.addOffset(0, controlPointsOffset, 0); }
  public static int createControlPointsVector(FlatBufferBuilder builder, int[] data) { builder.startVector(4, data.length, 4); for (int i = data.length - 1; i >= 0; i--) builder.addOffset(data[i]); return builder.endVector(); }
  public static void startControlPointsVector(FlatBufferBuilder builder, int numElems) { builder.startVector(4, numElems, 4); }
  public static int endVrepPath(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
  public static void finishVrepPathBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset); }
};

