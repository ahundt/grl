// automatically generated, do not modify

package grl.flatbuffer;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class FRI extends Table {
  public static FRI getRootAsFRI(ByteBuffer _bb) { return getRootAsFRI(_bb, new FRI()); }
  public static FRI getRootAsFRI(ByteBuffer _bb, FRI obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public FRI __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public byte overlayType() { int o = __offset(4); return o != 0 ? bb.get(o + bb_pos) : 1; }
  /**
   * Set the value for the send period of the connection from the KUKA controller to the remote side in [ms]. 
   * This means, the KUKA controller will send cyclic FRI messages every sendPeriod milliseconds to the remote side. 
   * 
   * 
   * Parameters:
   * sendPeriod - the send period in milliseconds, 1 <= sendPeriod <= 100. 
   * Note: The recommended value for good performance should be between 1-5 milliseconds. 
   */
  public int sendPeriodMillisec() { int o = __offset(6); return o != 0 ? bb.getInt(o + bb_pos) : 4; }
  /**
   * Set the receive multiplier of the cycle time from the remote side to the KUKA controller. 
   * This multiplier defines the value of the receivePeriod which is calculated:
   * receivePeriod = receiveMultiplier * sendPeriod 
   * 
   * The KUKA controller will expect a FRI response message every receivePeriod milliseconds from the remote side. 
   * 
   * The receivePeriod has to be within the range of:
   * 1 <= receivePeriod <= 100. 
   */
  public int setReceiveMultiplier() { int o = __offset(8); return o != 0 ? bb.getInt(o + bb_pos) : 5; }
  public boolean updatePortOnRemote() { int o = __offset(10); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  /**
   * Set the port ID of the socket at the controller side. 
   * Note: Do not change this port ID, unless your application requires different port IDs on both ends of the FRI channel.
   * For changing the FRI port ID on both sides, it is sufficient to call setPortOnRemote(int). 
   * Values of controllerPortID:  
   * "-1" - The configuration of setPortOnRemote(int) is used. This is the default. 
   * recommended range of port IDs: 30200 <= controllerPortID < 30210
   */
  public short portOnRemote() { int o = __offset(12); return o != 0 ? bb.getShort(o + bb_pos) : 0; }
  public boolean updatePortOnController() { int o = __offset(14); return o != 0 ? 0!=bb.get(o + bb_pos) : false; }
  /**
   *  Set the port ID of the FRI channel at the remote side. 
   *  By default, this port ID is used on both sides of the FRI channel, unless specified otherwise by setPortOnController(int). 
   *
   *  Values of portID: 
   *
   *  default port ID: 30200 
   *  recommended range of port IDs: 30200 <= portID < 30210 
   *  Since the FRI channel utilizes UDP as connection layer, make sure, that your network topology (firewall, network services) are chosen accordingly. 
   *
   *  Parameters:
   *  portID - the port ID > 0 (also known as UDP port number)
   */
  public short portOnController() { int o = __offset(16); return o != 0 ? bb.getShort(o + bb_pos) : 0; }

  public static int createFRI(FlatBufferBuilder builder,
      byte overlayType,
      int sendPeriodMillisec,
      int setReceiveMultiplier,
      boolean updatePortOnRemote,
      short portOnRemote,
      boolean updatePortOnController,
      short portOnController) {
    builder.startObject(7);
    FRI.addSetReceiveMultiplier(builder, setReceiveMultiplier);
    FRI.addSendPeriodMillisec(builder, sendPeriodMillisec);
    FRI.addPortOnController(builder, portOnController);
    FRI.addPortOnRemote(builder, portOnRemote);
    FRI.addUpdatePortOnController(builder, updatePortOnController);
    FRI.addUpdatePortOnRemote(builder, updatePortOnRemote);
    FRI.addOverlayType(builder, overlayType);
    return FRI.endFRI(builder);
  }

  public static void startFRI(FlatBufferBuilder builder) { builder.startObject(7); }
  public static void addOverlayType(FlatBufferBuilder builder, byte overlayType) { builder.addByte(0, overlayType, 1); }
  public static void addSendPeriodMillisec(FlatBufferBuilder builder, int sendPeriodMillisec) { builder.addInt(1, sendPeriodMillisec, 4); }
  public static void addSetReceiveMultiplier(FlatBufferBuilder builder, int setReceiveMultiplier) { builder.addInt(2, setReceiveMultiplier, 5); }
  public static void addUpdatePortOnRemote(FlatBufferBuilder builder, boolean updatePortOnRemote) { builder.addBoolean(3, updatePortOnRemote, false); }
  public static void addPortOnRemote(FlatBufferBuilder builder, short portOnRemote) { builder.addShort(4, portOnRemote, 0); }
  public static void addUpdatePortOnController(FlatBufferBuilder builder, boolean updatePortOnController) { builder.addBoolean(5, updatePortOnController, false); }
  public static void addPortOnController(FlatBufferBuilder builder, short portOnController) { builder.addShort(6, portOnController, 0); }
  public static int endFRI(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

