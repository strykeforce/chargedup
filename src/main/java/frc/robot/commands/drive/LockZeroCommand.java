package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class LockZeroCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;

  public LockZeroCommand(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.lockZero();
  }
}
