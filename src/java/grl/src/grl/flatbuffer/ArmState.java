// automatically generated, do not modify

package grl.flatbuffer;

public final class ArmState {
  private ArmState() { }
  public static final byte NONE = 0;
  public static final byte StartArm = 1;
  public static final byte StopArm = 2;
  public static final byte PauseArm = 3;
  public static final byte TeachArm = 4;
  public static final byte MoveArmTrajectory = 5;
  public static final byte MoveArmServo = 6;

  private static final String[] names = { "NONE", "StartArm", "StopArm", "PauseArm", "TeachArm", "MoveArmTrajectory", "MoveArmServo", };

  public static String name(int e) { return names[e]; }
};

