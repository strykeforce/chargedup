package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;

public class CommunityToDockCommandGroup extends SequentialCommandGroup {
  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;

  public CommunityToDockCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      HandSubsystem handSubsystem,
      ArmSubsystem armSubsystem,
      String pathOne,
      String pathTwo) {

    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
    addCommands(firstPath, secondPath);
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
  }
}
