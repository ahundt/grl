// automatically generated, do not modify

package grl.flatbuffer.kuka.iiwa;

import grl.flatbuffer.JointState;
import grl.flatbuffer.Pose;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class KUKAiiwaMonitorState extends Table {
  public static KUKAiiwaMonitorState getRootAsKUKAiiwaMonitorState(ByteBuffer _bb) { return getRootAsKUKAiiwaMonitorState(_bb, new KUKAiiwaMonitorState()); }
  public static KUKAiiwaMonitorState getRootAsKUKAiiwaMonitorState(ByteBuffer _bb, KUKAiiwaMonitorState obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public KUKAiiwaMonitorState __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public JointState measuredState() { return measuredState(new JointState()); }
  public JointState measuredState(JointState obj) { int o = __offset(4); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  public Pose cartesianFlangePose() { return cartesianFlangePose(new Pose()); }
  public Pose cartesianFlangePose(Pose obj) { int o = __offset(6); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public double jointTorques(int j) { int o = __offset(8); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int jointTorquesLength() { int o = __offset(8); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer jointTorquesAsByteBuffer() { return __vector_as_bytebuffer(8, 8); }
  public double externalJointTorques(int j) { int o = __offset(10); return o != 0 ? bb.getDouble(__vector(o) + j * 8) : 0; }
  public int externalJointTorquesLength() { int o = __offset(10); return o != 0 ? __vector_len(o) : 0; }
  public ByteBuffer externalJointTorquesAsByteBuffer() { return __vector_as_bytebuffer(10, 8); }
  public byte operationMode() { int o = __offset(12); return o != 0 ? bb.get(o + bb_pos) : 0; }

  public static void startKUKAiiwaMonitorState(FlatBufferBuilder builder) { builder.startObject(5); }
  public static void addMeasuredState(FlatBufferBuilder builder, int measuredStateOffset) { builder.addOffset(0, measuredStateOffset, 0); }
  public static void addCartesianFlangePose(FlatBufferBuilder builder, int cartesianFlangePoseOffset) { builder.addStruct(1, cartesianFlangePoseOffset, 0); }
  public static void addJointTorques(FlatBufferBuilder builder, int jointTorquesOffset) { builder.addOffset(2, jointTorquesOffset, 0); }
  public static int createJointTorquesVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startJointTorquesVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static void addExternalJointTorques(FlatBufferBuilder builder, int externalJointTorquesOffset) { builder.addOffset(3, externalJointTorquesOffset, 0); }
  public static int createExternalJointTorquesVector(FlatBufferBuilder builder, double[] data) { builder.startVector(8, data.length, 8); for (int i = data.length - 1; i >= 0; i--) builder.addDouble(data[i]); return builder.endVector(); }
  public static void startExternalJointTorquesVector(FlatBufferBuilder builder, int numElems) { builder.startVector(8, numElems, 8); }
  public static void addOperationMode(FlatBufferBuilder builder, byte operationMode) { builder.addByte(4, operationMode, 0); }
  public static int endKUKAiiwaMonitorState(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

