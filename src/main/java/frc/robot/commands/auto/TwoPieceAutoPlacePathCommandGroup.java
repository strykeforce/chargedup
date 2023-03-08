package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.robotState.AutoPlaceCommandGroup;
import frc.robot.commands.robotState.AutonFloorIntakeCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.ReleaseGamepieceCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.commands.vision.SetVisionUpdateCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

public class TwoPieceAutoPlacePathCommandGroup extends SequentialCommandGroup {
  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;

  public TwoPieceAutoPlacePathCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      IntakeSubsystem intakeSubsystem,
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
            new SetVisionUpdateCommand(driveSubsystem, false)),
        new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem),
        new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem),
        new ParallelCommandGroup(
            firstPath,
            new AutonFloorIntakeCommand(robotStateSubsystem, armSubsystem, intakeSubsystem)),
        new ParallelCommandGroup(
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CUBE),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH),
            secondPath,
            new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem)),
        new SetVisionUpdateCommand(driveSubsystem, true),
        new AutoPlaceCommandGroup(driveSubsystem, robotStateSubsystem, armSubsystem, handSubsystem),
        new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
  }
}
