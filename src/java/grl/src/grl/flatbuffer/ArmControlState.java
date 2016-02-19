// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class ArmControlState extends Table {
  public static ArmControlState getRootAsArmControlState(ByteBuffer _bb) { return getRootAsArmControlState(_bb, new ArmControlState()); }
  public static ArmControlState getRootAsArmControlState(ByteBuffer _bb, ArmControlState obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public ArmControlState __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public String name() { int o = __offset(4); return o != 0 ? __string(o + bb_pos) : null; }
  public ByteBuffer nameAsByteBuffer() { return __vector_as_bytebuffer(4, 1); }
  public long sequenceNumber() { int o = __offset(6); return o != 0 ? bb.getLong(o + bb_pos) : 0; }
  public double timeStamp() { int o = __offset(8); return o != 0 ? bb.getDouble(o + bb_pos) : 0; }
  public byte stateType() { int o = __offset(10); return o != 0 ? bb.get(o + bb_pos) : 0; }
  public Table state(Table obj) { int o = __offset(12); return o != 0 ? __union(obj, o) : null; }

  public static int createArmControlState(FlatBufferBuilder builder,
      int name,
      long sequenceNumber,
      double timeStamp,
      byte state_type,
      int state) {
    builder.startObject(5);
    ArmControlState.addTimeStamp(builder, timeStamp);
    ArmControlState.addSequenceNumber(builder, sequenceNumber);
    ArmControlState.addState(builder, state);
    ArmControlState.addName(builder, name);
    ArmControlState.addStateType(builder, state_type);
    return ArmControlState.endArmControlState(builder);
  }

  public static void startArmControlState(FlatBufferBuilder builder) { builder.startObject(5); }
  public static void addName(FlatBufferBuilder builder, int nameOffset) { builder.addOffset(0, nameOffset, 0); }
  public static void addSequenceNumber(FlatBufferBuilder builder, long sequenceNumber) { builder.addLong(1, sequenceNumber, 0); }
  public static void addTimeStamp(FlatBufferBuilder builder, double timeStamp) { builder.addDouble(2, timeStamp, 0); }
  public static void addStateType(FlatBufferBuilder builder, byte stateType) { builder.addByte(3, stateType, 0); }
  public static void addState(FlatBufferBuilder builder, int stateOffset) { builder.addOffset(4, stateOffset, 0); }
  public static int endArmControlState(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

