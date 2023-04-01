package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoPickupCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  Pose2d endPose;
  private ProfiledPIDController omegaAutoDriveController;
  private ProfiledPIDController xAutoDriveController;
  private ProfiledPIDController yAutoDriveController;
  private static final Logger logger = LoggerFactory.getLogger(AutoPickupCommand.class);

  public AutoPickupCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem, Pose2d endPose) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.endPose = endPose;
    omegaAutoDriveController =
        new ProfiledPIDController(
            DriveConstants.kPOmega,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmega));
    omegaAutoDriveController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    xAutoDriveController =
        new ProfiledPIDController(
            DriveConstants.kPAutoPickup,
            DriveConstants.kIAutoPickup,
            DriveConstants.kDAutoPickup,
            new TrapezoidProfile.Constraints(
                DriveConstants.kAutoPickupDriveMaxVel, DriveConstants.kAutoPickupDriveMaxAccel));

    yAutoDriveController =
        new ProfiledPIDController(
            DriveConstants.kPAutoPickup,
            DriveConstants.kIAutoPickup,
            DriveConstants.kDAutoPickup,
            new TrapezoidProfile.Constraints(
                DriveConstants.kAutoPickupDriveMaxVel, DriveConstants.kAutoPickupDriveMaxAccel));
  }

  @Override
  public void initialize() {
    endPose =
        new Pose2d(
            new Translation2d(
                (robotStateSubsystem.isBlueAlliance()
                    ? endPose.getX()
                    : RobotStateConstants.kFieldMaxX - endPose.getX()),
                endPose.getY()),
            new Rotation2d());
    logger.info("Starting auto pickup going to {}", endPose);
    omegaAutoDriveController.reset(driveSubsystem.getPoseMeters().getRotation().getRadians());
    xAutoDriveController.reset(
        driveSubsystem.getPoseMeters().getX(), driveSubsystem.getFieldRelSpeed().vxMetersPerSecond);
    yAutoDriveController.reset(
        driveSubsystem.getPoseMeters().getY(), driveSubsystem.getFieldRelSpeed().vyMetersPerSecond);
  }

  @Override
  public void execute() {
    double xCalc =
        xAutoDriveController.calculate(driveSubsystem.getPoseMeters().getX(), endPose.getX());
    double yCalc =
        yAutoDriveController.calculate(driveSubsystem.getPoseMeters().getY(), endPose.getY());
    double omegaCalc =
        omegaAutoDriveController.calculate(
            MathUtil.angleModulus(driveSubsystem.getGyroRotation2d().getRadians()),
            robotStateSubsystem.getAllianceColor() == Alliance.Blue ? 0.0 : Math.PI);
    driveSubsystem.move(xCalc, yCalc, omegaCalc, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getPoseMeters().getX() - endPose.getX())
            <= DriveConstants.kAutoPickupCloseEnough
        && Math.abs(driveSubsystem.getPoseMeters().getY() - endPose.getY())
            <= DriveConstants.kAutoPickupCloseEnough;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
    logger.info(
        "Done AutoPickup y error {} | x error {} | END POSE {}",
        Math.abs(endPose.getY() - driveSubsystem.getPoseMeters().getY()),
        Math.abs(endPose.getX() - driveSubsystem.getPoseMeters().getX()),
        driveSubsystem.getPoseMeters());
  }
}
