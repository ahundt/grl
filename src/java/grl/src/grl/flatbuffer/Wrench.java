// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Wrench extends Struct {
  public Wrench __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public Vector3d force() { return force(new Vector3d()); }
  public Vector3d force(Vector3d obj) { return obj.__init(bb_pos + 0, bb); }
  public Vector3d torque() { return torque(new Vector3d()); }
  public Vector3d torque(Vector3d obj) { return obj.__init(bb_pos + 24, bb); }
  public Vector3d forceOffset() { return forceOffset(new Vector3d()); }
  public Vector3d forceOffset(Vector3d obj) { return obj.__init(bb_pos + 48, bb); }

  public static int createWrench(FlatBufferBuilder builder, double force_x, double force_y, double force_z, double torque_x, double torque_y, double torque_z, double force_offset_x, double force_offset_y, double force_offset_z) {
    builder.prep(8, 72);
    builder.prep(8, 24);
    builder.putDouble(force_offset_z);
    builder.putDouble(force_offset_y);
    builder.putDouble(force_offset_x);
    builder.prep(8, 24);
    builder.putDouble(torque_z);
    builder.putDouble(torque_y);
    builder.putDouble(torque_x);
    builder.prep(8, 24);
    builder.putDouble(force_z);
    builder.putDouble(force_y);
    builder.putDouble(force_x);
    return builder.offset();
  }
};

