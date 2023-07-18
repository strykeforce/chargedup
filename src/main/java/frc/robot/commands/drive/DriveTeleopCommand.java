package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HandSubsystem.HandStates;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;
import java.util.function.DoubleSupplier;
import org.strykeforce.thirdcoast.util.ExpoScale;

public class DriveTeleopCommand extends CommandBase {
  private DoubleSupplier fwdStick;
  private DoubleSupplier strStick;
  private DoubleSupplier yawStick;
  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final HandSubsystem handSubsystem;
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
      DoubleSupplier fwdStick,
      DoubleSupplier strStick,
      DoubleSupplier yawStick,
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      HandSubsystem handSubsystem) {
    addRequirements(driveSubsystem);
    this.fwdStick = fwdStick;
    this.strStick = strStick;
    this.yawStick = yawStick;
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.handSubsystem = handSubsystem;
  }

  @Override
  public void execute() {
    rawValues[0] = fwdStick.getAsDouble();
    rawValues[1] = strStick.getAsDouble();
    rawValues[2] = yawStick.getAsDouble();
    double yawPercent = 1.0, movePercent = 1.0;
    if (robotStateSubsystem.getRobotState() == RobotState.AUTO_SHELF
        || robotStateSubsystem.getRobotState() == RobotState.MANUAL_SHELF
        || robotStateSubsystem.getRobotState() == RobotState.TO_MANUAL_SHELF
        || robotStateSubsystem.getRobotState() == RobotState.SHELF_WAIT_TRANSITION
        || robotStateSubsystem.getRobotState() == RobotState.AUTO_DRIVE) {
      yawPercent = DriveConstants.kShelfYawPercent;
      movePercent = DriveConstants.kShelfMovePercent;
    }

    if (robotStateSubsystem.getRobotState() == RobotState.AUTO_SCORE
        || robotStateSubsystem.getRobotState() == RobotState.MANUAL_SCORE
        || robotStateSubsystem.getRobotState() == RobotState.TO_MANUAL_SCORE
        || robotStateSubsystem.getRobotState() == RobotState.AUTO_DRIVE
        || robotStateSubsystem.getRobotState() == RobotState.RELEASE_GAME_PIECE
            && handSubsystem.getHandState() != HandStates.OPEN
        || robotStateSubsystem.getRobotState() == RobotState.CHECK_AMBIGUITY) {
      yawPercent = DriveConstants.kPlaceYawPercent;
      movePercent = DriveConstants.kPlaceMovePercent;
    }

    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue) {
      driveSubsystem.drive(
          movePercent
              * -DriveConstants.kMaxSpeedMetersPerSecond
              * fwdLimiter.calculate(
                  MathUtil.applyDeadband(fwdStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          movePercent
              * -DriveConstants.kMaxSpeedMetersPerSecond
              * strLimiter.calculate(
                  MathUtil.applyDeadband(strStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          yawPercent
              * -DriveConstants.kMaxOmega
              * yawLimiter.calculate(
                  MathUtil.applyDeadband(
                      yawStick.getAsDouble(), DriveConstants.kDeadbandAllStick)));
    } else if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      driveSubsystem.drive(
          movePercent
              * -DriveConstants.kMaxSpeedMetersPerSecond
              * fwdLimiter.calculate(
                  MathUtil.applyDeadband(
                      -fwdStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          movePercent
              * -DriveConstants.kMaxSpeedMetersPerSecond
              * strLimiter.calculate(
                  MathUtil.applyDeadband(
                      -strStick.getAsDouble(), DriveConstants.kDeadbandAllStick)),
          yawPercent
              * -DriveConstants.kMaxOmega
              * yawLimiter.calculate(
                  MathUtil.applyDeadband(
                      yawStick.getAsDouble(), DriveConstants.kDeadbandAllStick)));
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
}
