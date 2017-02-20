// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class TeachArm extends Table {
  public static TeachArm getRootAsTeachArm(ByteBuffer _bb) { return getRootAsTeachArm(_bb, new TeachArm()); }
  public static TeachArm getRootAsTeachArm(ByteBuffer _bb, TeachArm obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public TeachArm __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }


  public static void startTeachArm(FlatBufferBuilder builder) { builder.startObject(0); }
  public static int endTeachArm(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

