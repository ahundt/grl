// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class MoveArmServo extends Table {
  public static MoveArmServo getRootAsMoveArmServo(ByteBuffer _bb) { return getRootAsMoveArmServo(_bb, new MoveArmServo()); }
  public static MoveArmServo getRootAsMoveArmServo(ByteBuffer _bb, MoveArmServo obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public MoveArmServo __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public grl.flatbuffer.JointState goal() { return goal(new grl.flatbuffer.JointState()); }
  public grl.flatbuffer.JointState goal(grl.flatbuffer.JointState obj) { int o = __offset(4); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }

  public static int createMoveArmServo(FlatBufferBuilder builder,
      int goalOffset) {
    builder.startObject(1);
    MoveArmServo.addGoal(builder, goalOffset);
    return MoveArmServo.endMoveArmServo(builder);
  }

  public static void startMoveArmServo(FlatBufferBuilder builder) { builder.startObject(1); }
  public static void addGoal(FlatBufferBuilder builder, int goalOffset) { builder.addOffset(0, goalOffset, 0); }
  public static int endMoveArmServo(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

