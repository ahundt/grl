// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Inertia extends Struct {
  public Inertia __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public double mass() { return bb.getDouble(bb_pos + 0); }
  public Pose pose() { return pose(new Pose()); }
  public Pose pose(Pose obj) { return obj.__init(bb_pos + 8, bb); }
  public double ixx() { return bb.getDouble(bb_pos + 64); }
  public double ixy() { return bb.getDouble(bb_pos + 72); }
  public double ixz() { return bb.getDouble(bb_pos + 80); }
  public double iyy() { return bb.getDouble(bb_pos + 88); }
  public double iyz() { return bb.getDouble(bb_pos + 96); }
  public double izz() { return bb.getDouble(bb_pos + 104); }

  public static int createInertia(FlatBufferBuilder builder, double mass, double pose_position_x, double pose_position_y, double pose_position_z, double pose_orientation_x, double pose_orientation_y, double pose_orientation_z, double pose_orientation_w, double ixx, double ixy, double ixz, double iyy, double iyz, double izz) {
    builder.prep(8, 112);
    builder.putDouble(izz);
    builder.putDouble(iyz);
    builder.putDouble(iyy);
    builder.putDouble(ixz);
    builder.putDouble(ixy);
    builder.putDouble(ixx);
    builder.prep(8, 56);
    builder.prep(8, 32);
    builder.putDouble(pose_orientation_w);
    builder.putDouble(pose_orientation_z);
    builder.putDouble(pose_orientation_y);
    builder.putDouble(pose_orientation_x);
    builder.prep(8, 24);
    builder.putDouble(pose_position_z);
    builder.putDouble(pose_position_y);
    builder.putDouble(pose_position_x);
    builder.putDouble(mass);
    return builder.offset();
  }
};

