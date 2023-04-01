package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
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
import frc.robot.subsystems.VisionSubsystem;

public class ThreePieceSmoothAutoCommandGroup extends SequentialCommandGroup
    implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  DriveAutonCommand fourthPath;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private RobotStateSubsystem robotStateSubsystem;

  public ThreePieceSmoothAutoCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      VisionSubsystem visionSubsystem,
      String pathOne,
      String pathTwo,
      String pathThree,
      String pathFour) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, false, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, false, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, pathThree, false, false);
    fourthPath = new DriveAutonCommand(driveSubsystem, pathFour, true, false);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ParallelCommandGroup(
            new ZeroGyroCommand(driveSubsystem),
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CONE),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH),
            new ZeroElevatorCommand(elevatorSubsystem),
            new AutoGrabConeCommand(handSubsystem),
            new SetVisionUpdateCommand(driveSubsystem, false)),
        new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem),
        new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem),
        new ParallelDeadlineGroup(
            firstPath,
            new AutoFloorIntakeCommand(
                robotStateSubsystem, intakeSubsystem, armSubsystem, handSubsystem),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH)),
        new ParallelDeadlineGroup(
            secondPath,
            new SequentialCommandGroup(
                new PastXPositionCommand(robotStateSubsystem, driveSubsystem, 2.8),
                new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem))),
        new ParallelCommandGroup(
            new ShootGamepieceCommand(handSubsystem, robotStateSubsystem),
            new ClearGamePieceCommand(
                robotStateSubsystem)) /*, //Q do we need clear gamepiece command?
                                      // Cube 2
                                      new ParallelDeadlineGroup(
                                          thirdPath,
                                          new AutoFloorIntakeCommand(
                                              robotStateSubsystem, intakeSubsystem, armSubsystem, handSubsystem),
                                          new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.MID)),
                                      new ParallelDeadlineGroup(
                                          fourthPath,
                                          new SequentialCommandGroup(
                                              new PastXPositionCommand(robotStateSubsystem, driveSubsystem, 2.8),
                                              new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem))),
                                      new ParallelCommandGroup(
                                          new ShootGamepieceCommand(handSubsystem, robotStateSubsystem), //Q can shoot and clear gamepiece be parallel
                                          new ClearGamePieceCommand(robotStateSubsystem),
                                          new SetVisionUpdateCommand(driveSubsystem, true))*/);

    /*,new ParallelCommandGroup(
        fourthPath,
        new PastXPositionCommand(
            robotStateSubsystem, driveSubsystem, Constants.AutonConstants.kPastXPosition)),
    new AutoPlaceCommandGroup(driveSubsystem, robotStateSubsystem, armSubsystem, handSubsystem),
    new ClearGamePieceCommand(robotStateSubsystem)*/
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    fourthPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
