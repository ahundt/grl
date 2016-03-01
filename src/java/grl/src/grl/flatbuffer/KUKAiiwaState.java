// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class KUKAiiwaState extends Table {
  public static KUKAiiwaState getRootAsKUKAiiwaState(ByteBuffer _bb) { return getRootAsKUKAiiwaState(_bb, new KUKAiiwaState()); }
  public static KUKAiiwaState getRootAsKUKAiiwaState(ByteBuffer _bb, KUKAiiwaState obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public KUKAiiwaState __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public String name() { int o = __offset(4); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer nameAsByteBuffer() { return __vector_as_bytebuffer(4, 1); }
  public String destination() { int o = __offset(6); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer destinationAsByteBuffer() { return __vector_as_bytebuffer(6, 1); }
  public String source() { int o = __offset(8); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer sourceAsByteBuffer() { return __vector_as_bytebuffer(8, 1); }
  public double timestamp() { int o = __offset(10); return o != 0 ? bb.getDouble(o + bb_pos) : 0; }
  public boolean setArmControlState() { int o = __offset(12); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  public ArmControlState armControlState() { return armControlState(new ArmControlState()); }
  public ArmControlState armControlState(ArmControlState obj) { int o = __offset(14); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  public boolean setArmConfiguration() { int o = __offset(16); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  public KUKAiiwaArmConfiguration armConfiguration() { return armConfiguration(new KUKAiiwaArmConfiguration()); }
  public KUKAiiwaArmConfiguration armConfiguration(KUKAiiwaArmConfiguration obj) { int o = __offset(18); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  public boolean hasMonitorState() { int o = __offset(20); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  public KUKAiiwaMonitorState monitorState() { return monitorState(new KUKAiiwaMonitorState()); }
  public KUKAiiwaMonitorState monitorState(KUKAiiwaMonitorState obj) { int o = __offset(22); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  public boolean hasMonitorConfig() { int o = __offset(24); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  public KUKAiiwaMonitorConfiguration monitorConfig() { return monitorConfig(new KUKAiiwaMonitorConfiguration()); }
  public KUKAiiwaMonitorConfiguration monitorConfig(KUKAiiwaMonitorConfiguration obj) { int o = __offset(26); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }

  public static int createKUKAiiwaState(FlatBufferBuilder builder,
      int name,
      int destination,
      int source,
      double timestamp,
      boolean setArmControlState,
      int armControlState,
      boolean setArmConfiguration,
      int armConfiguration,
      boolean hasMonitorState,
      int monitorState,
      boolean hasMonitorConfig,
      int monitorConfig) {
    builder.startObject(12);
    KUKAiiwaState.addTimestamp(builder, timestamp);
    KUKAiiwaState.addMonitorConfig(builder, monitorConfig);
    KUKAiiwaState.addMonitorState(builder, monitorState);
    KUKAiiwaState.addArmConfiguration(builder, armConfiguration);
    KUKAiiwaState.addArmControlState(builder, armControlState);
    KUKAiiwaState.addSource(builder, source);
    KUKAiiwaState.addDestination(builder, destination);
    KUKAiiwaState.addName(builder, name);
    KUKAiiwaState.addHasMonitorConfig(builder, hasMonitorConfig);
    KUKAiiwaState.addHasMonitorState(builder, hasMonitorState);
    KUKAiiwaState.addSetArmConfiguration(builder, setArmConfiguration);
    KUKAiiwaState.addSetArmControlState(builder, setArmControlState);
    return KUKAiiwaState.endKUKAiiwaState(builder);
  }

  public static void startKUKAiiwaState(FlatBufferBuilder builder) { builder.startObject(12); }
  public static void addName(FlatBufferBuilder builder, int nameOffset) { builder.addOffset(0, nameOffset, 0); }
  public static void addDestination(FlatBufferBuilder builder, int destinationOffset) { builder.addOffset(1, destinationOffset, 0); }
  public static void addSource(FlatBufferBuilder builder, int sourceOffset) { builder.addOffset(2, sourceOffset, 0); }
  public static void addTimestamp(FlatBufferBuilder builder, double timestamp) { builder.addDouble(3, timestamp, 0); }
  public static void addSetArmControlState(FlatBufferBuilder builder, boolean setArmControlState) { builder.addBoolean(4, setArmControlState, false); }
  public static void addArmControlState(FlatBufferBuilder builder, int armControlStateOffset) { builder.addOffset(5, armControlStateOffset, 0); }
  public static void addSetArmConfiguration(FlatBufferBuilder builder, boolean setArmConfiguration) { builder.addBoolean(6, setArmConfiguration, false); }
  public static void addArmConfiguration(FlatBufferBuilder builder, int armConfigurationOffset) { builder.addOffset(7, armConfigurationOffset, 0); }
  public static void addHasMonitorState(FlatBufferBuilder builder, boolean hasMonitorState) { builder.addBoolean(8, hasMonitorState, false); }
  public static void addMonitorState(FlatBufferBuilder builder, int monitorStateOffset) { builder.addOffset(9, monitorStateOffset, 0); }
  public static void addHasMonitorConfig(FlatBufferBuilder builder, boolean hasMonitorConfig) { builder.addBoolean(10, hasMonitorConfig, false); }
  public static void addMonitorConfig(FlatBufferBuilder builder, int monitorConfigOffset) { builder.addOffset(11, monitorConfigOffset, 0); }
  public static int endKUKAiiwaState(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

