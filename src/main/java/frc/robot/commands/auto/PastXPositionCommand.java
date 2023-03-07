package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;

public class PastXPositionCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private double xPose;
  private final RobotStateSubsystem robotStateSubsystem;

  public PastXPositionCommand(
      RobotStateSubsystem robotStateSubsystem, DriveSubsystem driveSubsystem, double xPose) {
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.xPose = xPose;
  }

  @Override
  public boolean isFinished() {
    if (robotStateSubsystem.isBlueAlliance()) {
      return driveSubsystem.getPoseMeters().getX() <= xPose;
    } else {
      return driveSubsystem.getPoseMeters().getX() >= Constants.FieldConstants.kFieldLength - xPose;
    }
  }
}
