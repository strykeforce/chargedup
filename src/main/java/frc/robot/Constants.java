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
    public static final int kIntakeFalconID = 0; // FIXME: replace wih actual id
    public static final int kRollerFalconID = 0; // FIXME: replace wih actual id
    public static final int kIntakeExtendFalconID = 0; // FIXME: replace wih actual id

    public static final int kCloseEnoughTicks = 150;
    public static final int kExtendPosTicks = 0; // FIXME: replace with correct # of ticks
    public static final int kRetractPosTicks = 0; // FIXME: replace with correct # of ticks

    public static final int kIntakeZeroTicks = 2800; // FIXME: insert actual value
    public static final int kZeroStableCounts = 3;
    public static final int kZeroStableBand = 20;
  }
}
