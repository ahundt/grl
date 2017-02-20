// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Time extends Struct {
  public Time __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double sec() { return bb.getDouble(bb_pos + 0); }

  public static int createTime(FlatBufferBuilder builder, double sec) {
    builder.prep(8, 8);
    builder.putDouble(sec);
    return builder.offset();
  }
};

