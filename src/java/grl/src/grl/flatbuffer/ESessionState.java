// automatically generated, do not modify

package grl.flatbuffer;

public final class ESessionState {
  private ESessionState() { }
  /**
   * No session
   */
  public static final byte IDLE = 0;
  /**
   * Monitoring mode, receive state but connection too inconsistent to command
   */
  public static final byte MONITORING_WAIT = 1;
  /**
   * Monitoring mode 
   */
  public static final byte MONITORING_READY = 2;
  /**
   * About to command (Overlay created in Java interface)
   */
  public static final byte COMMANDING_WAIT = 3;
  /**
   * Actively commanding the arm with FRI
   */
  public static final byte COMMANDING_ACTIVE = 4;

  private static final String[] names = { "IDLE", "MONITORING_WAIT", "MONITORING_READY", "COMMANDING_WAIT", "COMMANDING_ACTIVE", };

  public static String name(int e) { return names[e]; }
};

