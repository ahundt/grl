// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Quaternion extends Struct {
  public Quaternion __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double x() { return bb.getDouble(bb_pos + 0); }
  public double y() { return bb.getDouble(bb_pos + 8); }
  public double z() { return bb.getDouble(bb_pos + 16); }
  public double w() { return bb.getDouble(bb_pos + 24); }

  public static int createQuaternion(FlatBufferBuilder builder, double x, double y, double z, double w) {
    builder.prep(8, 32);
    builder.putDouble(w);
    builder.putDouble(z);
    builder.putDouble(y);
    builder.putDouble(x);
    return builder.offset();
  }
};

