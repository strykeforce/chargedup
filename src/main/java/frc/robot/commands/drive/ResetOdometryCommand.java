package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometryCommand extends InstantCommand {
  DriveSubsystem driveSubsystem;

  public ResetOdometryCommand(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(DriveConstants.kOdometryZeroPos);
  }
}
