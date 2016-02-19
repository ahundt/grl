// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class SmartServo extends Table {
  public static SmartServo getRootAsSmartServo(ByteBuffer _bb) { return getRootAsSmartServo(_bb, new SmartServo()); }
  public static SmartServo getRootAsSmartServo(ByteBuffer _bb, SmartServo obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public SmartServo __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double jointAccelerationRel(int j) { int o = __offset(4); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int jointAccelerationRelLength() { int o = __offset(4); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer jointAccelerationRelAsByteBuffer() { return __vector_as_bytebuffer(4, 8); }
  public double jointVelocityRel(int j) { int o = __offset(6); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int jointVelocityRelLength() { int o = __offset(6); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer jointVelocityRelAsByteBuffer() { return __vector_as_bytebuffer(6, 8); }

  public static int createSmartServo(FlatBufferBuilder builder,
      int jointAccelerationRel,
      int jointVelocityRel) {
    builder.startObject(2);
    SmartServo.addJointVelocityRel(builder, jointVelocityRel);
    SmartServo.addJointAccelerationRel(builder, jointAccelerationRel);
    return SmartServo.endSmartServo(builder);
  }

  public static void startSmartServo(FlatBufferBuilder builder) { builder.startObject(2); }
  public static void addJointAccelerationRel(FlatBufferBuilder builder, int jointAccelerationRelOffset) { builder.addOffset(0, jointAccelerationRelOffset, 0); }
  public static int createJointAccelerationRelVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startJointAccelerationRelVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static void addJointVelocityRel(FlatBufferBuilder builder, int jointVelocityRelOffset) { builder.addOffset(1, jointVelocityRelOffset, 0); }
  public static int createJointVelocityRelVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startJointVelocityRelVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static int endSmartServo(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

