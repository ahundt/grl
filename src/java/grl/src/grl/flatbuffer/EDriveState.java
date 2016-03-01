// automatically generated, do not modify

package grl.flatbuffer;

public final class EDriveState {
  private EDriveState() { }
  /**
   * Driving mode currently unused
   */
  public static final byte OFF = 1;
  /**
   * About to drive
   */
  public static final byte TRANSITIONING = 2;
  /**
   * Actively commanding arm
   */
  public static final byte ACTIVE = 3;

  private static final String[] names = { "OFF", "TRANSITIONING", "ACTIVE", };

  public static String name(int e) { return names[e - OFF]; }
};

