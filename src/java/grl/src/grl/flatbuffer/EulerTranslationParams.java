// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class EulerTranslationParams extends Table {
  public static EulerTranslationParams getRootAsEulerTranslationParams(ByteBuffer _bb) { return getRootAsEulerTranslationParams(_bb, new EulerTranslationParams()); }
  public static EulerTranslationParams getRootAsEulerTranslationParams(ByteBuffer _bb, EulerTranslationParams obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public EulerTranslationParams __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double x() { int o = __offset(4); return o != 0 ? bb.getDouble(o + bb_pos) : 0; }
  public double y() { int o = __offset(6); return o != 0 ? bb.getDouble(o + bb_pos) : 0; }
  public double z() { int o = __offset(8); return o != 0 ? bb.getDouble(o + bb_pos) : 0; }

  public static int createEulerTranslationParams(FlatBufferBuilder builder,
      double x,
      double y,
      double z) {
    builder.startObject(3);
    EulerTranslationParams.addZ(builder, z);
    EulerTranslationParams.addY(builder, y);
    EulerTranslationParams.addX(builder, x);
    return EulerTranslationParams.endEulerTranslationParams(builder);
  }

  public static void startEulerTranslationParams(FlatBufferBuilder builder) { builder.startObject(3); }
  public static void addX(FlatBufferBuilder builder, double x) { builder.addDouble(0, x, 0); }
  public static void addY(FlatBufferBuilder builder, double y) { builder.addDouble(1, y, 0); }
  public static void addZ(FlatBufferBuilder builder, double z) { builder.addDouble(2, z, 0); }
  public static int endEulerTranslationParams(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

