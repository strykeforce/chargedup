package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveStates;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveToPlaceNotPathCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(DriveToPlaceNotPathCommand.class);
  private TargetCol targetCol;

  public DriveToPlaceNotPathCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    // if (robotStateSubsystem.isBlueAlliance()) {
    //   if (driveSubsystem.getPoseMeters().getX() < RobotStateConstants.kFieldMaxX / 2)
    //     driveSubsystem.driveToPose(
    //         robotStateSubsystem.getAutoPlaceDriveTarget(
    //             driveSubsystem.getPoseMeters().getY(), robotStateSubsystem.getTargetCol()));
    //   else driveSubsystem.driveToPose(new Pose2d());
    // } else {
    //   if (driveSubsystem.getPoseMeters().getX() > RobotStateConstants.kFieldMaxX / 2)
    //     driveSubsystem.driveToPose(
    //         robotStateSubsystem.getAutoPlaceDriveTarget(
    //             driveSubsystem.getPoseMeters().getY(), robotStateSubsystem.getTargetCol()));
    //   else driveSubsystem.driveToPose(new Pose2d());
    // }
    // Pose2d endPose =
    //     robotStateSubsystem.getAutoPlaceDriveTarget(
    //         driveSubsystem.getPoseMeters().getY(), robotStateSubsystem.getTargetCol());
    driveSubsystem.driveToPose(robotStateSubsystem.getTargetCol());
    logger.info("Moving to place Drive.");
  }

  @Override
  public boolean isFinished() {
    return driveSubsystem.isAutoDriveFinished()
        || driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FAILED;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setAutoDriving(false);
    driveSubsystem.drive(0, 0, 0);
    // driveSubsystem.grapherTrajectoryActive(false);
    logger.info("AutoDrive Command Ended"); // timer.get());
  }
}
