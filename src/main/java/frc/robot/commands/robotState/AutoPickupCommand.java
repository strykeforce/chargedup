package frc.robot.commands.robotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HandConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoPickupCommand extends CommandBase {
  private static final Logger logger = LoggerFactory.getLogger(AutoPickupCommand.class);
  public DriveSubsystem driveSubsystem;
  public RobotStateSubsystem robotStateSubsystem;
  public Pose2d endPose;

  public AutoPickupCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem, Pose2d endPose) {
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.endPose = endPose;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.drivePickup(endPose);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getPoseMeters().getX() - endPose.getX())
                <= DriveConstants.kAutoPickupCloseEnough
            && Math.abs(driveSubsystem.getPoseMeters().getY() - endPose.getY())
                <= DriveConstants.kAutoPickupCloseEnough
        || robotStateSubsystem.handStableCount() > HandConstants.kHasCubeStableCountsAuto;
  }

  @Override
  public void end(boolean interrupted) {
    robotStateSubsystem.setLights(0, 0, 0);
    driveSubsystem.drive(0, 0, 0);
    logger.info(
        "Done AutoPickup y error {} | x error {} | END POSE {} | interrupted: {}",
        Math.abs(endPose.getY() - driveSubsystem.getPoseMeters().getY()),
        Math.abs(endPose.getX() - driveSubsystem.getPoseMeters().getX()),
        driveSubsystem.getPoseMeters(),
        interrupted);
  }
}
