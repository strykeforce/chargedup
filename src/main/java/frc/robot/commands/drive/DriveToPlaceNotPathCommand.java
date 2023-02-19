package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveToPlaceNotPathCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);

  public DriveToPlaceNotPathCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetHolonomicController();
    if (robotStateSubsystem.isBlueAlliance()) {
      if (driveSubsystem.getPoseMeters().getX() < RobotStateConstants.kFieldMaxX / 2)
        driveSubsystem.driveToPose(
            robotStateSubsystem.getAutoPlaceDriveTarget(driveSubsystem.getPoseMeters().getY()));
      else driveSubsystem.driveToPose(new Pose2d());
    } else {
      if (driveSubsystem.getPoseMeters().getX() > RobotStateConstants.kFieldMaxX / 2)
        driveSubsystem.driveToPose(
            robotStateSubsystem.getAutoPlaceDriveTarget(driveSubsystem.getPoseMeters().getY()));
      else driveSubsystem.driveToPose(new Pose2d());
    }
    logger.info("Moving to place");
  }

  @Override
  public boolean isFinished() {
    return !driveSubsystem.autoDriving;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDriving = false;
    driveSubsystem.setEnableHolo(false);
    driveSubsystem.drive(0, 0, 0);
    // driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}"); // timer.get());
  }
}
