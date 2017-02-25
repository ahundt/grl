// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Pose extends Struct {
  public Pose __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public Vector3d position() { return position(new Vector3d()); }
  public Vector3d position(Vector3d obj) { return obj.__init(bb_pos + 0, bb); }
  public Quaternion orientation() { return orientation(new Quaternion()); }
  public Quaternion orientation(Quaternion obj) { return obj.__init(bb_pos + 24, bb); }

  public static int createPose(FlatBufferBuilder builder, double position_x, double position_y, double position_z, double orientation_x, double orientation_y, double orientation_z, double orientation_w) {
    builder.prep(8, 56);
    builder.prep(8, 32);
    builder.putDouble(orientation_w);
    builder.putDouble(orientation_z);
    builder.putDouble(orientation_y);
    builder.putDouble(orientation_x);
    builder.prep(8, 24);
    builder.putDouble(position_z);
    builder.putDouble(position_y);
    builder.putDouble(position_x);
    return builder.offset();
  }
};

