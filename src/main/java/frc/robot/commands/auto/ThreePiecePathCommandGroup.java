package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ThreePiecePathCommandGroup extends SequentialCommandGroup {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  DriveAutonCommand fourthPath;

  public ThreePiecePathCommandGroup(
      DriveSubsystem driveSubsystem,
      String pathOne,
      String pathTwo,
      String pathThree,
      String pathFour) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, pathThree, true, false);
    fourthPath = new DriveAutonCommand(driveSubsystem, pathFour, true, false);
    addCommands(firstPath, secondPath, thirdPath, fourthPath);
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    fourthPath.generateTrajectory();
  }
}
