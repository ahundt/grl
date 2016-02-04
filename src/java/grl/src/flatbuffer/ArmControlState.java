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

  public long sequenceNumber() { int o = __offset(4); return o != 0 ? bb.getLong(o + bb_pos) : 0; }
  public double timeStamp() { int o = __offset(6); return o != 0 ? bb.getDouble(o + bb_pos) : 0; }
  public byte stateType() { int o = __offset(8); return o != 0 ? bb.get(o + bb_pos) : 0; }
  public Table state(Table obj) { int o = __offset(10); return o != 0 ? __union(obj, o) : null; }

  public static int createArmControlState(FlatBufferBuilder builder,
      long sequenceNumber,
      double timeStamp,
      byte state_type,
      int stateOffset) {
    builder.startObject(4);
    ArmControlState.addTimeStamp(builder, timeStamp);
    ArmControlState.addSequenceNumber(builder, sequenceNumber);
    ArmControlState.addState(builder, stateOffset);
    ArmControlState.addStateType(builder, state_type);
    return ArmControlState.endArmControlState(builder);
  }

  public static void startArmControlState(FlatBufferBuilder builder) { builder.startObject(4); }
  public static void addSequenceNumber(FlatBufferBuilder builder, long sequenceNumber) { builder.addLong(0, sequenceNumber, 0); }
  public static void addTimeStamp(FlatBufferBuilder builder, double timeStamp) { builder.addDouble(1, timeStamp, 0); }
  public static void addStateType(FlatBufferBuilder builder, byte stateType) { builder.addByte(2, stateType, 0); }
  public static void addState(FlatBufferBuilder builder, int stateOffset) { builder.addOffset(3, stateOffset, 0); }
  public static int endArmControlState(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

