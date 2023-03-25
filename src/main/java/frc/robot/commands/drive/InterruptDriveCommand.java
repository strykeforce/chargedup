package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class InterruptDriveCommand extends InstantCommand {

  public InterruptDriveCommand(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
  }
}
