package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
  public static final class IntakeConstants {
    // FIXME: need correct values

    public static final int kIntakeFalconID = 0;
    public static final int kExtendTalonID = 0;

    public static final int kCloseEnoughTicks = 150;
    public static final int kExtendPosTicks = 0;
    public static final int kRetractPosTicks = 0;

    public static final double kIntakeSpeed = 0.5;
    public static final double kIntakeEjectSpeed = -0.5;
    public static final double kEjectTimerDelaySec = 3;

    public static final int kIntakeZeroTicks = 2800;
    public static final int kZeroStableCounts = 3;
    public static final int kZeroStableBand = 20;
  }
}
