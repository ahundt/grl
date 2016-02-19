// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Object extends Table {
  public static Object getRootAsObject(ByteBuffer _bb) { return getRootAsObject(_bb, new Object()); }
  public static Object getRootAsObject(ByteBuffer _bb, Object obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public Object __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public String name() { int o = __offset(4); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer nameAsByteBuffer() { return __vector_as_bytebuffer(4, 1); }
  public String parent() { int o = __offset(6); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer parentAsByteBuffer() { return __vector_as_bytebuffer(6, 1); }
  public Pose pose() { return pose(new Pose()); }
  public Pose pose(Pose obj) { int o = __offset(8); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public Inertia inertia() { return inertia(new Inertia()); }
  public Inertia inertia(Inertia obj) { int o = __offset(10); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }

  public static void startObject(FlatBufferBuilder builder) { builder.startObject(4); }
  public static void addName(FlatBufferBuilder builder, int nameOffset) { builder.addOffset(0, nameOffset, 0); }
  public static void addParent(FlatBufferBuilder builder, int parentOffset) { builder.addOffset(1, parentOffset, 0); }
  public static void addPose(FlatBufferBuilder builder, int poseOffset) { builder.addStruct(2, poseOffset, 0); }
  public static void addInertia(FlatBufferBuilder builder, int inertiaOffset) { builder.addStruct(3, inertiaOffset, 0); }
  public static int endObject(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

