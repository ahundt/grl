// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
/**
 * "ProcessData" is a field that appears 
 * on your physical kuka tablet. 
 * This message allows you to update these
 * fields on the tablet yourself.
 */
public final class ProcessData extends Table {
  public static ProcessData getRootAsProcessData(ByteBuffer _bb) { return getRootAsProcessData(_bb, new ProcessData()); }
  public static ProcessData getRootAsProcessData(ByteBuffer _bb, ProcessData obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public ProcessData __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public String dataType() { int o = __offset(4); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer dataTypeAsByteBuffer() { return __vector_as_bytebuffer(4, 1); }
  public String defaultValue() { int o = __offset(6); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer defaultValueAsByteBuffer() { return __vector_as_bytebuffer(6, 1); }
  public String displayName() { int o = __offset(8); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer displayNameAsByteBuffer() { return __vector_as_bytebuffer(8, 1); }
  public String id() { int o = __offset(10); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer idAsByteBuffer() { return __vector_as_bytebuffer(10, 1); }
  public String min() { int o = __offset(12); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer minAsByteBuffer() { return __vector_as_bytebuffer(12, 1); }
  public String max() { int o = __offset(14); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer maxAsByteBuffer() { return __vector_as_bytebuffer(14, 1); }
  public String unit() { int o = __offset(16); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer unitAsByteBuffer() { return __vector_as_bytebuffer(16, 1); }
  public String value() { int o = __offset(18); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer valueAsByteBuffer() { return __vector_as_bytebuffer(18, 1); }
  /**
   * should the data be removed completely?
   */
  public boolean shouldRemove() { int o = __offset(20); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  /**
   * should the data be updated to these values? 
   */
  public boolean shouldUpdate() { int o = __offset(22); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }

  public static int createProcessData(FlatBufferBuilder builder,
      int dataType,
      int defaultValue,
      int displayName,
      int id,
      int min,
      int max,
      int unit,
      int value,
      boolean shouldRemove,
      boolean shouldUpdate) {
    builder.startObject(10);
    ProcessData.addValue(builder, value);
    ProcessData.addUnit(builder, unit);
    ProcessData.addMax(builder, max);
    ProcessData.addMin(builder, min);
    ProcessData.addId(builder, id);
    ProcessData.addDisplayName(builder, displayName);
    ProcessData.addDefaultValue(builder, defaultValue);
    ProcessData.addDataType(builder, dataType);
    ProcessData.addShouldUpdate(builder, shouldUpdate);
    ProcessData.addShouldRemove(builder, shouldRemove);
    return ProcessData.endProcessData(builder);
  }

  public static void startProcessData(FlatBufferBuilder builder) { builder.startObject(10); }
  public static void addDataType(FlatBufferBuilder builder, int dataTypeOffset) { builder.addOffset(0, dataTypeOffset, 0); }
  public static void addDefaultValue(FlatBufferBuilder builder, int defaultValueOffset) { builder.addOffset(1, defaultValueOffset, 0); }
  public static void addDisplayName(FlatBufferBuilder builder, int displayNameOffset) { builder.addOffset(2, displayNameOffset, 0); }
  public static void addId(FlatBufferBuilder builder, int idOffset) { builder.addOffset(3, idOffset, 0); }
  public static void addMin(FlatBufferBuilder builder, int minOffset) { builder.addOffset(4, minOffset, 0); }
  public static void addMax(FlatBufferBuilder builder, int maxOffset) { builder.addOffset(5, maxOffset, 0); }
  public static void addUnit(FlatBufferBuilder builder, int unitOffset) { builder.addOffset(6, unitOffset, 0); }
  public static void addValue(FlatBufferBuilder builder, int valueOffset) { builder.addOffset(7, valueOffset, 0); }
  public static void addShouldRemove(FlatBufferBuilder builder, boolean shouldRemove) { builder.addBoolean(8, shouldRemove, false); }
  public static void addShouldUpdate(FlatBufferBuilder builder, boolean shouldUpdate) { builder.addBoolean(9, shouldUpdate, false); }
  public static int endProcessData(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

