// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class ArmConfiguration extends Table {
  public static ArmConfiguration getRootAsArmConfiguration(ByteBuffer _bb) { return getRootAsArmConfiguration(_bb, new ArmConfiguration()); }
  public static ArmConfiguration getRootAsArmConfiguration(ByteBuffer _bb, ArmConfiguration obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public ArmConfiguration __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public grl.flatbuffer.Object tool() { return tool(new grl.flatbuffer.Object()); }
  public grl.flatbuffer.Object tool(grl.flatbuffer.Object obj) { int o = __offset(4); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }

  public static int createArmConfiguration(FlatBufferBuilder builder,
      int toolOffset) {
    builder.startObject(1);
    ArmConfiguration.addTool(builder, toolOffset);
    return ArmConfiguration.endArmConfiguration(builder);
  }

  public static void startArmConfiguration(FlatBufferBuilder builder) { builder.startObject(1); }
  public static void addTool(FlatBufferBuilder builder, int toolOffset) { builder.addOffset(0, toolOffset, 0); }
  public static int endArmConfiguration(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

