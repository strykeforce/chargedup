package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.xLockCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.ReleaseGamepieceCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.commands.vision.SetVisionUpdateCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

public class CommunityToDockCommandGroup extends SequentialCommandGroup {
  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;

  public CommunityToDockCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      HandSubsystem handSubsystem,
      ArmSubsystem armSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      String pathOne,
      String pathTwo) {

    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);

    addCommands(
      new ParallelCommandGroup(
        new SetGamePieceCommand(robotStateSubsystem, GamePiece.CONE),
        new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH),
        new ZeroElevatorCommand(elevatorSubsystem),
        new AutoGrabConeCommand(handSubsystem),
        new SetVisionUpdateCommand(driveSubsystem, false)
      ),
      new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem),
      new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem),
      firstPath, 
      secondPath,
      new xLockCommand(driveSubsystem));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
  }
}
