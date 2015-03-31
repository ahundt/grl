// automatically generated, do not modify

package flatbuffers;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public class VrepControlPoint extends Table {
  public static VrepControlPoint getRootAsVrepControlPoint(ByteBuffer _bb) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (new VrepControlPoint()).__init(_bb.getInt(_bb.position()) + _bb.position(), _bb); }
  public VrepControlPoint __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public Vector3d position() { return position(new Vector3d()); }
  public Vector3d position(Vector3d obj) { int o = __offset(4); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public EulerXYZd rotation() { return rotation(new EulerXYZd()); }
  public EulerXYZd rotation(EulerXYZd obj) { int o = __offset(6); return o != 0 ? obj.__init(o + bb_pos, bb) : null; }
  public double relativeVelocity() { int o = __offset(8); return o != 0 ? bb.getDouble(o + bb_pos) : 1.0; }
  public int bezierPointCount() { int o = __offset(10); return o != 0 ? bb.getInt(o + bb_pos) : 1; }
  public double interpolationFactor1() { int o = __offset(12); return o != 0 ? bb.getDouble(o + bb_pos) : 0.5; }
  public double interpolationFactor2() { int o = __offset(14); return o != 0 ? bb.getDouble(o + bb_pos) : 0.5; }
  public double virtualDistance() { int o = __offset(16); return o != 0 ? bb.getDouble(o + bb_pos) : 0.0; }
  public int auxiliaryFlags() { int o = __offset(18); return o != 0 ? bb.getInt(o + bb_pos) : 0; }
  public double auxiliaryChannel1() { int o = __offset(20); return o != 0 ? bb.getDouble(o + bb_pos) : 0.0; }
  public double auxiliaryChannel2() { int o = __offset(22); return o != 0 ? bb.getDouble(o + bb_pos) : 0.0; }
  public double auxiliaryChannel3() { int o = __offset(24); return o != 0 ? bb.getDouble(o + bb_pos) : 0.0; }
  public double auxiliaryChannel4() { int o = __offset(26); return o != 0 ? bb.getDouble(o + bb_pos) : 0.0; }

  public static void startVrepControlPoint(FlatBufferBuilder builder) { builder.startObject(12); }
  public static void addPosition(FlatBufferBuilder builder, int positionOffset) { builder.addStruct(0, positionOffset, 0); }
  public static void addRotation(FlatBufferBuilder builder, int rotationOffset) { builder.addStruct(1, rotationOffset, 0); }
  public static void addRelativeVelocity(FlatBufferBuilder builder, double relativeVelocity) { builder.addDouble(2, relativeVelocity, 1.0); }
  public static void addBezierPointCount(FlatBufferBuilder builder, int bezierPointCount) { builder.addInt(3, bezierPointCount, 1); }
  public static void addInterpolationFactor1(FlatBufferBuilder builder, double interpolationFactor1) { builder.addDouble(4, interpolationFactor1, 0.5); }
  public static void addInterpolationFactor2(FlatBufferBuilder builder, double interpolationFactor2) { builder.addDouble(5, interpolationFactor2, 0.5); }
  public static void addVirtualDistance(FlatBufferBuilder builder, double virtualDistance) { builder.addDouble(6, virtualDistance, 0.0); }
  public static void addAuxiliaryFlags(FlatBufferBuilder builder, int auxiliaryFlags) { builder.addInt(7, auxiliaryFlags, 0); }
  public static void addAuxiliaryChannel1(FlatBufferBuilder builder, double auxiliaryChannel1) { builder.addDouble(8, auxiliaryChannel1, 0.0); }
  public static void addAuxiliaryChannel2(FlatBufferBuilder builder, double auxiliaryChannel2) { builder.addDouble(9, auxiliaryChannel2, 0.0); }
  public static void addAuxiliaryChannel3(FlatBufferBuilder builder, double auxiliaryChannel3) { builder.addDouble(10, auxiliaryChannel3, 0.0); }
  public static void addAuxiliaryChannel4(FlatBufferBuilder builder, double auxiliaryChannel4) { builder.addDouble(11, auxiliaryChannel4, 0.0); }
  public static int endVrepControlPoint(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
  public static void finishVrepControlPointBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset); }
};

