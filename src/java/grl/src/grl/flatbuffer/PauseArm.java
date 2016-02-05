// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class PauseArm extends Table {
  public static PauseArm getRootAsPauseArm(ByteBuffer _bb) { return getRootAsPauseArm(_bb, new PauseArm()); }
  public static PauseArm getRootAsPauseArm(ByteBuffer _bb, PauseArm obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public PauseArm __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }


  public static void startPauseArm(FlatBufferBuilder builder) { builder.startObject(0); }
  public static int endPauseArm(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

