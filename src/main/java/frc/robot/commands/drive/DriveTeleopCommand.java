package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;
import org.strykeforce.thirdcoast.util.ExpoScale;

public class DriveTeleopCommand extends CommandBase {
  private final Joystick joystick;
  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private double[] rawValues = new double[3];
  private final ExpoScale expoScaleYaw =
      new ExpoScale(DriveConstants.kDeadbandAllStick, DriveConstants.kExpoScaleYawFactor);
  // private final RateLimit rateLimitYaw = new RateLimit(DriveConstants.kRateLimitYaw);
  // private final RateLimit rateLimitMove = new RateLimit(DriveConstants.kRateLimitMove);
  // private double[] adjustedValues = new double[3];
  // private final double vectorOffset =
  //         / (DriveConstants.kExpoScaleMoveFactor
  //                 * Math.pow((Math.sqrt(2) - DriveConstants.kDeadbandAllStick), 3)
  //             + (1 - DriveConstants.kExpoScaleMoveFactor)
  //                 * (Math.sqrt(2) - DriveConstants.kDeadbandAllStick));
  private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
  private final SlewRateLimiter strLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
  private final SlewRateLimiter yawLimiter = new SlewRateLimiter(DriveConstants.kRateLimitYaw);

  public DriveTeleopCommand(
      Joystick driver, DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    joystick = driver;
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void execute() {
    rawValues[0] = joystick.getRawAxis(RobotContainer.Axis.LEFT_X.id);
    rawValues[1] = joystick.getRawAxis(RobotContainer.Axis.LEFT_Y.id);
    rawValues[2] = joystick.getRawAxis(RobotContainer.Axis.RIGHT_Y.id);
    double yawPercent = 1.0, movePercent = 1.0;
    if (robotStateSubsystem.getRobotState() == RobotState.AUTO_SHELF
        || robotStateSubsystem.getRobotState() == RobotState.MANUAL_SHELF
        || robotStateSubsystem.getRobotState() == RobotState.TO_MANUAL_SHELF
        || robotStateSubsystem.getRobotState() == RobotState.SHELF_WAIT_TRANSITION
        || robotStateSubsystem.getRobotState() == RobotState.AUTO_DRIVE) {
      yawPercent = DriveConstants.kShelfYawPercent;
      movePercent = DriveConstants.kShelfMovePercent;
    }

    if ((robotStateSubsystem.getRobotState() == RobotState.AUTO_SCORE
            || robotStateSubsystem.getRobotState() == RobotState.MANUAL_SCORE
            || robotStateSubsystem.getRobotState() == RobotState.TO_MANUAL_SCORE
            || robotStateSubsystem.getRobotState() == RobotState.AUTO_DRIVE
            || robotStateSubsystem.getRobotState() == RobotState.RELEASE_GAME_PIECE
            || robotStateSubsystem.getRobotState() == RobotState.CHECK_AMBIGUITY)
        && robotStateSubsystem.getTargetLevel() != TargetLevel.LOW) {
      yawPercent = DriveConstants.kPlaceYawPercent;
      movePercent = DriveConstants.kPlaceMovePercent;
    }

    // adjustedValues =
    //     calcAdjustedValues(
    //         joystick.getRawAxis(RobotContainer.Axis.LEFT_X.id),
    //         joystick.getRawAxis(RobotContainer.Axis.LEFT_Y.id),
    //         joystick.getRawAxis(RobotContainer.Axis.RIGHT_Y.id));

    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue) {
      driveSubsystem.drive(
          movePercent
              * -DriveConstants.kMaxSpeedMetersPerSecond
              * fwdLimiter.calculate(
                  MathUtil.applyDeadband(
                      joystick.getRawAxis(RobotContainer.Axis.LEFT_X.id),
                      DriveConstants.kDeadbandAllStick)),
          movePercent
              * -DriveConstants.kMaxSpeedMetersPerSecond
              * strLimiter.calculate(
                  MathUtil.applyDeadband(
                      joystick.getRawAxis(RobotContainer.Axis.LEFT_Y.id),
                      DriveConstants.kDeadbandAllStick)),
          yawPercent
              * -DriveConstants.kMaxOmega
              * yawLimiter.calculate(
                  MathUtil.applyDeadband(
                      joystick.getRawAxis(RobotContainer.Axis.RIGHT_Y.id),
                      DriveConstants.kDeadbandAllStick)));
    } else if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      driveSubsystem.drive(
          movePercent
              * -DriveConstants.kMaxSpeedMetersPerSecond
              * fwdLimiter.calculate(
                  MathUtil.applyDeadband(
                      -joystick.getRawAxis(RobotContainer.Axis.LEFT_X.id),
                      DriveConstants.kDeadbandAllStick)),
          movePercent
              * -DriveConstants.kMaxSpeedMetersPerSecond
              * strLimiter.calculate(
                  MathUtil.applyDeadband(
                      -joystick.getRawAxis(RobotContainer.Axis.LEFT_Y.id),
                      DriveConstants.kDeadbandAllStick)),
          yawPercent
              * -DriveConstants.kMaxOmega
              * yawLimiter.calculate(
                  MathUtil.applyDeadband(
                      joystick.getRawAxis(RobotContainer.Axis.RIGHT_Y.id),
                      DriveConstants.kDeadbandAllStick)));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }

  // private double applyExpoScaling(double input) {
  //   double y;

  //   if (Math.abs(input) < DriveConstants.kDeadbandAllStick) {
  //     return 0;
  //   }

  //   y =
  //       input > 0
  //           ? input - DriveConstants.kDeadbandAllStick
  //           : input + DriveConstants.kDeadbandAllStick;

  //   return (DriveConstants.kExpoScaleMoveFactor * Math.pow(y, 3)
  //           + (1 - DriveConstants.kExpoScaleMoveFactor) * y)
  //       * vectorOffset;
  // }

  // private double[] calcAdjustedValues(double rawForward, double rawStrafe, double rawYaw) {
  //   double[] tempAdjustedValues = new double[3];
  //   double rawAngle = Math.atan2(rawForward, rawStrafe);
  //   double orgMag = (Math.sqrt(Math.pow(rawForward, 2) + Math.pow(rawStrafe, 2)));
  //   double adjustedMag = applyExpoScaling(orgMag);
  //   tempAdjustedValues[0] = Math.sin(rawAngle) * adjustedMag;
  //   tempAdjustedValues[1] = Math.cos(rawAngle) * adjustedMag;
  //   tempAdjustedValues[2] = expoScaleYaw.apply(rawYaw);

  //   return tempAdjustedValues;
  // }
}
