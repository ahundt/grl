// automatically generated, do not modify

package flatbuffers;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

public class JointState extends Table {
  public static JointState getRootAsJointState(ByteBuffer _bb) { return getRootAsJointState(_bb, new JointState()); }
  public static JointState getRootAsJointState(ByteBuffer _bb, JointState obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public JointState __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double position(int j) { int o = __offset(4); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int positionLength() { int o = __offset(4); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer positionAsByteBuffer() { return __vector_as_bytebuffer(4, 8); }
  public double velocity(int j) { int o = __offset(6); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int velocityLength() { int o = __offset(6); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer velocityAsByteBuffer() { return __vector_as_bytebuffer(6, 8); }
  public double acceleration(int j) { int o = __offset(8); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int accelerationLength() { int o = __offset(8); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer accelerationAsByteBuffer() { return __vector_as_bytebuffer(8, 8); }

  public static int createJointState(FlatBufferBuilder builder,
      int position,
      int velocity,
      int acceleration) {
    builder.startObject(3);
    JointState.addAcceleration(builder, acceleration);
    JointState.addVelocity(builder, velocity);
    JointState.addPosition(builder, position);
    return JointState.endJointState(builder);
  }

  public static void startJointState(FlatBufferBuilder builder) { builder.startObject(3); }
  public static void addPosition(FlatBufferBuilder builder, int positionOffset) { builder.addOffset(0, positionOffset, 0); }
  public static int createPositionVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startPositionVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static void addVelocity(FlatBufferBuilder builder, int velocityOffset) { builder.addOffset(1, velocityOffset, 0); }
  public static int createVelocityVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startVelocityVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static void addAcceleration(FlatBufferBuilder builder, int accelerationOffset) { builder.addOffset(2, accelerationOffset, 0); }
  public static int createAccelerationVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startAccelerationVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static int endJointState(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
  public static void finishJointStateBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset); }
};

