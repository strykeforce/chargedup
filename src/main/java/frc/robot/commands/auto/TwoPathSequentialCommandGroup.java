package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TwoPathSequentialCommandGroup extends SequentialCommandGroup {
  public TwoPathSequentialCommandGroup(
      DriveSubsystem driveSubsystem, String pathOne, String pathTwo) {
    addCommands(
        new DriveAutonCommand(driveSubsystem, pathOne, true, true),
        new DriveAutonCommand(driveSubsystem, pathTwo, true, false));
  }
}
