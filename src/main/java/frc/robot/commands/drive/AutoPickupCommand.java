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
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.VisionSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoPickupCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  Pose2d desiredPose;
  Pose2d endPose;
  private ProfiledPIDController omegaAutoDriveController;
  private ProfiledPIDController xAutoDriveController;
  private ProfiledPIDController yAutoDriveController;
  private VisionSubsystem visionSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(AutoPickupCommand.class);

  public AutoPickupCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      Pose2d endPose,
      VisionSubsystem visionSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.visionSubsystem = visionSubsystem;
    desiredPose = endPose;
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
                    ? desiredPose.getX()
                    : RobotStateConstants.kFieldMaxX - desiredPose.getX()),
                desiredPose.getY()),
            new Rotation2d());
    if (visionSubsystem.isCameraWorking())
      visionSubsystem.resetOdometry(endPose, robotStateSubsystem.isBlueAlliance());
    logger.info("Starting auto pickup going to {}", endPose);
    omegaAutoDriveController.reset(driveSubsystem.getPoseMeters().getRotation().getRadians());
    xAutoDriveController.reset(
        driveSubsystem.getPoseMeters().getX(),
        robotStateSubsystem.isBlueAlliance()
            ? driveSubsystem.getFieldRelSpeed().vxMetersPerSecond
            : -driveSubsystem.getFieldRelSpeed().vxMetersPerSecond);
    yAutoDriveController.reset(
        driveSubsystem.getPoseMeters().getY(),driveSubsystem.getFieldRelSpeed().vyMetersPerSecond);
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
    if (robotStateSubsystem.isBlueAlliance()) {
      if (driveSubsystem.getPoseMeters().getX() <= endPose.getX()) {
        xCalc = Math.abs(xCalc);
      }
    } else if (driveSubsystem.getPoseMeters().getX() >= endPose.getX())
      xCalc = -1 * Math.abs(xCalc);
    logger.info(
        "Moving X : {} | Moving Y : {} | \nCurrent Pose {}",
        xCalc,
        yCalc,
        driveSubsystem.getPoseMeters());
    logger.info(
        "X controller: err: {}, goal: {}\n",
        xAutoDriveController.getPositionError(),
        xAutoDriveController.getGoal().position);
    driveSubsystem.move(xCalc, yCalc, omegaCalc, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getPoseMeters().getX() - endPose.getX())
                <= DriveConstants.kAutoPickupCloseEnough
            && Math.abs(driveSubsystem.getPoseMeters().getY() - endPose.getY())
                <= DriveConstants.kAutoPickupCloseEnough
        || robotStateSubsystem.getGamePiece() == GamePiece.CUBE;
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
