// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class CartesianImpedenceControlMode extends Table {
  public static CartesianImpedenceControlMode getRootAsCartesianImpedenceControlMode(ByteBuffer _bb) { return getRootAsCartesianImpedenceControlMode(_bb, new CartesianImpedenceControlMode()); }
  public static CartesianImpedenceControlMode getRootAsCartesianImpedenceControlMode(ByteBuffer _bb, CartesianImpedenceControlMode obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public CartesianImpedenceControlMode __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  /**
   * actual stiffness to set rot:[nm/rad]
   */
  public grl.flatbuffer.EulerPose stiffness() { return stiffness(new grl.flatbuffer.EulerPose()); }
  public grl.flatbuffer.EulerPose stiffness(grl.flatbuffer.EulerPose obj) { int o = __offset(4); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  /**
   * actual damping to set
   */
  public grl.flatbuffer.EulerPose damping() { return damping(new grl.flatbuffer.EulerPose()); }
  public grl.flatbuffer.EulerPose damping(grl.flatbuffer.EulerPose obj) { int o = __offset(6); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  /**
   * [Nm/rad] must be => 0.0
   */
  public double nullspaceStiffness() { int o = __offset(8); return o != 0 ? bb.getDouble(o + bb_pos) : 0; }
  /**
   * must be between 0.3-1.0 suggested is 0.7
   */
  public double nullspaceDamping() { int o = __offset(10); return o != 0 ? bb.getDouble(o + bb_pos) : 0; }
  /**
   * maximum deviation from set goal in mm and radians
   */
  public grl.flatbuffer.EulerPose maxPathDeviation() { return maxPathDeviation(new grl.flatbuffer.EulerPose()); }
  public grl.flatbuffer.EulerPose maxPathDeviation(grl.flatbuffer.EulerPose obj) { int o = __offset(12); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  /**
   * trans: [mm/s] rot: [rad/s]
   */
  public grl.flatbuffer.EulerPose maxCartesianVelocity() { return maxCartesianVelocity(new grl.flatbuffer.EulerPose()); }
  public grl.flatbuffer.EulerPose maxCartesianVelocity(grl.flatbuffer.EulerPose obj) { int o = __offset(14); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  /**
   * xyz: Newtons rpy:Nm (all >=0)
   */
  public grl.flatbuffer.EulerPose maxControlForce() { return maxControlForce(new grl.flatbuffer.EulerPose()); }
  public grl.flatbuffer.EulerPose maxControlForce(grl.flatbuffer.EulerPose obj) { int o = __offset(16); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  /**
   * stop if max control force is exceeded
   */
  public boolean maxControlForceExceededStop() { int o = __offset(18); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }

  public static void startCartesianImpedenceControlMode(FlatBufferBuilder builder) { builder.startObject(8); }
  public static void addStiffness(FlatBufferBuilder builder, int stiffnessOffset) { builder.addStruct(0, stiffnessOffset, 0); }
  public static void addDamping(FlatBufferBuilder builder, int dampingOffset) { builder.addStruct(1, dampingOffset, 0); }
  public static void addNullspaceStiffness(FlatBufferBuilder builder, double nullspaceStiffness) { builder.addDouble(2, nullspaceStiffness, 0); }
  public static void addNullspaceDamping(FlatBufferBuilder builder, double nullspaceDamping) { builder.addDouble(3, nullspaceDamping, 0); }
  public static void addMaxPathDeviation(FlatBufferBuilder builder, int maxPathDeviationOffset) { builder.addStruct(4, maxPathDeviationOffset, 0); }
  public static void addMaxCartesianVelocity(FlatBufferBuilder builder, int maxCartesianVelocityOffset) { builder.addStruct(5, maxCartesianVelocityOffset, 0); }
  public static void addMaxControlForce(FlatBufferBuilder builder, int maxControlForceOffset) { builder.addStruct(6, maxControlForceOffset, 0); }
  public static void addMaxControlForceExceededStop(FlatBufferBuilder builder, boolean maxControlForceExceededStop) { builder.addBoolean(7, maxControlForceExceededStop, false); }
  public static int endCartesianImpedenceControlMode(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

