// automatically generated, do not modify

package grl.flatbuffer;

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
  public JointState jointStateReal() { return jointStateReal(new JointState()); }
  public JointState jointStateReal(JointState obj) { int o = __offset(8); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  /**
   * FRI can adjust the java commanded position. "Interpolated" is the original Java commanded position.
   */
  public JointState jointStateInterpolated() { return jointStateInterpolated(new JointState()); }
  public JointState jointStateInterpolated(JointState obj) { int o = __offset(10); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  /**
   * The state of the arm as calculated by kuka after 
   * subtracting the known weights of the arm
   * and any attachments configured to be present.
   *
   * Most likely only contains torque.
   */
  public JointState externalState() { return externalState(new JointState()); }
  public JointState externalState(JointState obj) { int o = __offset(12); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  public byte operationMode() { int o = __offset(14); return o != 0 ? bb.get(o + bb_pos) : 0; }

  public static void startKUKAiiwaMonitorState(FlatBufferBuilder builder) { builder.startObject(6); }
  public static void addMeasuredState(FlatBufferBuilder builder, int measuredStateOffset) { builder.addOffset(0, measuredStateOffset, 0); }
  public static void addCartesianFlangePose(FlatBufferBuilder builder, int cartesianFlangePoseOffset) { builder.addStruct(1, cartesianFlangePoseOffset, 0); }
  public static void addJointStateReal(FlatBufferBuilder builder, int jointStateRealOffset) { builder.addOffset(2, jointStateRealOffset, 0); }
  public static void addJointStateInterpolated(FlatBufferBuilder builder, int jointStateInterpolatedOffset) { builder.addOffset(3, jointStateInterpolatedOffset, 0); }
  public static void addExternalState(FlatBufferBuilder builder, int externalStateOffset) { builder.addOffset(4, externalStateOffset, 0); }
  public static void addOperationMode(FlatBufferBuilder builder, byte operationMode) { builder.addByte(5, operationMode, 0); }
  public static int endKUKAiiwaMonitorState(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

