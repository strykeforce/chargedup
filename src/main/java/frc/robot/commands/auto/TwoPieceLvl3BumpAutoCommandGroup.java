package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.elevator.AutonZeroElevatorCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.robotState.ClearGamePieceCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.ReleaseGamepieceCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.commands.robotState.ShootGamepieceCommand;
import frc.robot.commands.vision.SetVisionUpdateCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

public class TwoPieceLvl3BumpAutoCommandGroup extends SequentialCommandGroup
    implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private RobotStateSubsystem robotStateSubsystem;

  public TwoPieceLvl3BumpAutoCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      String pathOne,
      String pathTwo,
      String pathThree) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, pathThree, true, false);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ParallelCommandGroup(
            new ZeroGyroCommand(driveSubsystem),
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CONE),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH),
            new AutonZeroElevatorCommand(elevatorSubsystem),
            new AutoGrabConeCommand(handSubsystem),
            new SetVisionUpdateCommand(driveSubsystem, false)),
        new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem),
        new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem),
        new ParallelCommandGroup(
            firstPath,
            new AutoFloorIntakeCommand(
                robotStateSubsystem, intakeSubsystem, armSubsystem, handSubsystem),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH)),
        new ParallelCommandGroup(
            new SequentialCommandGroup(secondPath, thirdPath),
            new SequentialCommandGroup(
                new PastXPositionCommand(
                    robotStateSubsystem, driveSubsystem, Constants.AutonConstants.kPastXPosition),
                new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem))),
        new ShootGamepieceCommand(handSubsystem, robotStateSubsystem, true),
        new ParallelCommandGroup(
            new ClearGamePieceCommand(robotStateSubsystem),
            new SetVisionUpdateCommand(driveSubsystem, true)));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
