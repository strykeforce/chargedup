package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoBalanceCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.drive.xLockCommand;
import frc.robot.commands.elevator.AutonZeroElevatorCommand;
import frc.robot.commands.robotState.ClearGamePieceCommand;
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
import frc.robot.subsystems.VisionSubsystem;

public class MiddleToDockWithMobility extends SequentialCommandGroup
    implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  DriveAutonCommand fourthPath;
  DriveAutonCommand fallbackPath;
  DriveAutonCommand fallbackPath2;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private RobotStateSubsystem robotStateSubsystem;

  public MiddleToDockWithMobility(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      VisionSubsystem visionSubsystem,
      String pathOne,
      String pathTwo) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, false, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
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
        new WaitCommand(1.0),
        firstPath,
        new WaitCommand(0.75),
        new ParallelRaceGroup(
            new AutoWaitForMatchTimeCommand(0.1),
            new SequentialCommandGroup(
                secondPath, new AutoBalanceCommand(false, driveSubsystem, robotStateSubsystem))),
        new xLockCommand(driveSubsystem));
    new ParallelCommandGroup(
        new ClearGamePieceCommand(robotStateSubsystem),
        new SetVisionUpdateCommand(driveSubsystem, true));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
