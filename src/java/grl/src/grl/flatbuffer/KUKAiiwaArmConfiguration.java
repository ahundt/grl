// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class KUKAiiwaArmConfiguration extends Table {
  public static KUKAiiwaArmConfiguration getRootAsKUKAiiwaArmConfiguration(ByteBuffer _bb) { return getRootAsKUKAiiwaArmConfiguration(_bb, new KUKAiiwaArmConfiguration()); }
  public static KUKAiiwaArmConfiguration getRootAsKUKAiiwaArmConfiguration(ByteBuffer _bb, KUKAiiwaArmConfiguration obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public KUKAiiwaArmConfiguration __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public String name() { int o = __offset(4); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer nameAsByteBuffer() { return __vector_as_bytebuffer(4, 1); }
  public short commandInterface() { int o = __offset(6); return o != 0 ? bb.getShort(o + bb_pos) : 0; }
  public short monitorInterface() { int o = __offset(8); return o != 0 ? bb.getShort(o + bb_pos) : 0; }
  public byte clientCommandMode() { int o = __offset(10); return o != 0 ? bb.get(o + bb_pos) : 0; }
  public byte overlayType() { int o = __offset(12); return o != 0 ? bb.get(o + bb_pos) : 0; }
  public byte controlMode() { int o = __offset(14); return o != 0 ? bb.get(o + bb_pos) : 0; }
  public SmartServo smartServoConfig() { return smartServoConfig(new SmartServo()); }
  public SmartServo smartServoConfig(SmartServo obj) { int o = __offset(16); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  public FRI FRIConfig() { return FRIConfig(new FRI()); }
  public FRI FRIConfig(FRI obj) { int o = __offset(18); return o != 0 ? obj.__init(__indirect(o + bb_pos), bb) : null; }
  public LinkObject tools(int j) { return tools(new LinkObject(), j); }
  public LinkObject tools(LinkObject obj, int j) { int o = __offset(20); return o != 0 ? obj.__init(__indirect(__vector(o) + j * 4), bb) : null; }
  public int toolsLength() { int o = __offset(20); return o != 0 ? __vector_len(o) : 0; }
  public ProcessData processData(int j) { return processData(new ProcessData(), j); }
  public ProcessData processData(ProcessData obj, int j) { int o = __offset(22); return o != 0 ? obj.__init(__indirect(__vector(o) + j * 4), bb) : null; }
  public int processDataLength() { int o = __offset(22); return o != 0 ? __vector_len(o) : 0; }
  public String currentMotionCenter() { int o = __offset(24); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer currentMotionCenterAsByteBuffer() { return __vector_as_bytebuffer(24, 1); }

  public static int createKUKAiiwaArmConfiguration(FlatBufferBuilder builder,
      int name,
      short commandInterface,
      short monitorInterface,
      byte clientCommandMode,
      byte overlayType,
      byte controlMode,
      int smartServoConfig,
      int FRIConfig,
      int tools,
      int processData,
      int currentMotionCenter) {
    builder.startObject(11);
    KUKAiiwaArmConfiguration.addCurrentMotionCenter(builder, currentMotionCenter);
    KUKAiiwaArmConfiguration.addProcessData(builder, processData);
    KUKAiiwaArmConfiguration.addTools(builder, tools);
    KUKAiiwaArmConfiguration.addFRIConfig(builder, FRIConfig);
    KUKAiiwaArmConfiguration.addSmartServoConfig(builder, smartServoConfig);
    KUKAiiwaArmConfiguration.addName(builder, name);
    KUKAiiwaArmConfiguration.addMonitorInterface(builder, monitorInterface);
    KUKAiiwaArmConfiguration.addCommandInterface(builder, commandInterface);
    KUKAiiwaArmConfiguration.addControlMode(builder, controlMode);
    KUKAiiwaArmConfiguration.addOverlayType(builder, overlayType);
    KUKAiiwaArmConfiguration.addClientCommandMode(builder, clientCommandMode);
    return KUKAiiwaArmConfiguration.endKUKAiiwaArmConfiguration(builder);
  }

  public static void startKUKAiiwaArmConfiguration(FlatBufferBuilder builder) { builder.startObject(11); }
  public static void addName(FlatBufferBuilder builder, int nameOffset) { builder.addOffset(0, nameOffset, 0); }
  public static void addCommandInterface(FlatBufferBuilder builder, short commandInterface) { builder.addShort(1, commandInterface, 0); }
  public static void addMonitorInterface(FlatBufferBuilder builder, short monitorInterface) { builder.addShort(2, monitorInterface, 0); }
  public static void addClientCommandMode(FlatBufferBuilder builder, byte clientCommandMode) { builder.addByte(3, clientCommandMode, 0); }
  public static void addOverlayType(FlatBufferBuilder builder, byte overlayType) { builder.addByte(4, overlayType, 0); }
  public static void addControlMode(FlatBufferBuilder builder, byte controlMode) { builder.addByte(5, controlMode, 0); }
  public static void addSmartServoConfig(FlatBufferBuilder builder, int smartServoConfigOffset) { builder.addOffset(6, smartServoConfigOffset, 0); }
  public static void addFRIConfig(FlatBufferBuilder builder, int FRIConfigOffset) { builder.addOffset(7, FRIConfigOffset, 0); }
  public static void addTools(FlatBufferBuilder builder, int toolsOffset) { builder.addOffset(8, toolsOffset, 0); }
  public static int createToolsVector(FlatBufferBuilder builder, int[] data) { builder.startVector(4, data.length, 4); for (int i = data.length - 1; i >= 0; i--) builder.addOffset(data[i]); return builder.endVector(); }
  public static void startToolsVector(FlatBufferBuilder builder, int numElems) { builder.startVector(4, numElems, 4); }
  public static void addProcessData(FlatBufferBuilder builder, int processDataOffset) { builder.addOffset(9, processDataOffset, 0); }
  public static int createProcessDataVector(FlatBufferBuilder builder, int[] data) { builder.startVector(4, data.length, 4); for (int i = data.length - 1; i >= 0; i--) builder.addOffset(data[i]); return builder.endVector(); }
  public static void startProcessDataVector(FlatBufferBuilder builder, int numElems) { builder.startVector(4, numElems, 4); }
  public static void addCurrentMotionCenter(FlatBufferBuilder builder, int currentMotionCenterOffset) { builder.addOffset(10, currentMotionCenterOffset, 0); }
  public static int endKUKAiiwaArmConfiguration(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
  public static void finishKUKAiiwaArmConfigurationBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset); }
};

