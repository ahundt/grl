// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class MoveArmCartesianServo extends Table {
  public static MoveArmCartesianServo getRootAsMoveArmCartesianServo(ByteBuffer _bb) { return getRootAsMoveArmCartesianServo(_bb, new MoveArmCartesianServo()); }
  public static MoveArmCartesianServo getRootAsMoveArmCartesianServo(ByteBuffer _bb, MoveArmCartesianServo obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public MoveArmCartesianServo __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public String parent() { int o = __offset(4); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer parentAsByteBuffer() { return __vector_as_bytebuffer(4, 1); }
  public Pose goal() { return goal(new Pose()); }
  public Pose goal(Pose obj) { int o = __offset(6); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }

  public static void startMoveArmCartesianServo(FlatBufferBuilder builder) { builder.startObject(2); }
  public static void addParent(FlatBufferBuilder builder, int parentOffset) { builder.addOffset(0, parentOffset, 0); }
  public static void addGoal(FlatBufferBuilder builder, int goalOffset) { builder.addStruct(1, goalOffset, 0); }
  public static int endMoveArmCartesianServo(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

