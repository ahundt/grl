// automatically generated, do not modify

package grl.flatbuffer.kuka.iiwa;

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
  public byte commandInterfaceType() { int o = __offset(6); return o != 0 ? bb.get(o + bb_pos) : 0; }
  public Table commandInterface(Table obj) { int o = __offset(8); return o != 0 ? __union(obj, o) : null; }
  public byte monitorInterfaceType() { int o = __offset(10); return o != 0 ? bb.get(o + bb_pos) : 0; }
  public Table monitorInterface(Table obj) { int o = __offset(12); return o != 0 ? __union(obj, o) : null; }
  public Object tools(int j) { return tools(new Object(), j); }
  public Object tools(Object obj, int j) { int o = __offset(14); return o != 0 ? obj.__init(__indirect(__vector(o) + j * 4), bb) : null; }
  public int toolsLength() { int o = __offset(14); return o != 0 ? __vector_len(o) : 0; }
  public String currentMotionCenter() { int o = __offset(16); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer currentMotionCenterAsByteBuffer() { return __vector_as_bytebuffer(16, 1); }

  public static int createKUKAiiwaArmConfiguration(FlatBufferBuilder builder,
      int name,
      byte commandInterface_type,
      int commandInterface,
      byte monitorInterface_type,
      int monitorInterface,
      int tools,
      int currentMotionCenter) {
    builder.startObject(7);
    KUKAiiwaArmConfiguration.addCurrentMotionCenter(builder, currentMotionCenter);
    KUKAiiwaArmConfiguration.addTools(builder, tools);
    KUKAiiwaArmConfiguration.addMonitorInterface(builder, monitorInterface);
    KUKAiiwaArmConfiguration.addCommandInterface(builder, commandInterface);
    KUKAiiwaArmConfiguration.addName(builder, name);
    KUKAiiwaArmConfiguration.addMonitorInterfaceType(builder, monitorInterface_type);
    KUKAiiwaArmConfiguration.addCommandInterfaceType(builder, commandInterface_type);
    return KUKAiiwaArmConfiguration.endKUKAiiwaArmConfiguration(builder);
  }

  public static void startKUKAiiwaArmConfiguration(FlatBufferBuilder builder) { builder.startObject(7); }
  public static void addName(FlatBufferBuilder builder, int nameOffset) { builder.addOffset(0, nameOffset, 0); }
  public static void addCommandInterfaceType(FlatBufferBuilder builder, byte commandInterfaceType) { builder.addByte(1, commandInterfaceType, 0); }
  public static void addCommandInterface(FlatBufferBuilder builder, int commandInterfaceOffset) { builder.addOffset(2, commandInterfaceOffset, 0); }
  public static void addMonitorInterfaceType(FlatBufferBuilder builder, byte monitorInterfaceType) { builder.addByte(3, monitorInterfaceType, 0); }
  public static void addMonitorInterface(FlatBufferBuilder builder, int monitorInterfaceOffset) { builder.addOffset(4, monitorInterfaceOffset, 0); }
  public static void addTools(FlatBufferBuilder builder, int toolsOffset) { builder.addOffset(5, toolsOffset, 0); }
  public static int createToolsVector(FlatBufferBuilder builder, int[] data) { builder.startVector(4, data.length, 4); for (int i = data.length - 1; i >= 0; i--) builder.addOffset(data[i]); return builder.endVector(); }
  public static void startToolsVector(FlatBufferBuilder builder, int numElems) { builder.startVector(4, numElems, 4); }
  public static void addCurrentMotionCenter(FlatBufferBuilder builder, int currentMotionCenterOffset) { builder.addOffset(6, currentMotionCenterOffset, 0); }
  public static int endKUKAiiwaArmConfiguration(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

