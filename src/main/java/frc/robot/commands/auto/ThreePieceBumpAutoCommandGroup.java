package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.commands.drive.AutoPickupCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.elevator.AutonZeroElevatorCommand;
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

public class ThreePieceBumpAutoCommandGroup extends SequentialCommandGroup
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

  public ThreePieceBumpAutoCommandGroup(
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
      String pathFour,
      String pathFallback) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, false, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, false, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, pathThree, false, false);
    fourthPath = new DriveAutonCommand(driveSubsystem, pathFour, false, false);
    fallbackPath = new DriveAutonCommand(driveSubsystem, pathFallback, true, false);
    fallbackPath2 = new DriveAutonCommand(driveSubsystem, pathFallback, true, false);
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
        new ParallelDeadlineGroup(
            firstPath,
            new AutoFloorIntakeCommand(
                robotStateSubsystem, intakeSubsystem, armSubsystem, handSubsystem),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH)),
        new ParallelCommandGroup(
            new SetVisionUpdateCommand(driveSubsystem, true),
            new AutoPickupCommand( // NOT USING JACOBS AUTOPICK IN DRIVESUBSYSTEM
                    driveSubsystem,
                    robotStateSubsystem,
                    RobotStateConstants.kCubeTwoAutoPickup,
                    visionSubsystem)
                .withTimeout(0.75)),
        new ParallelDeadlineGroup(
            secondPath,
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CUBE),
            new SetVisionUpdateCommand(driveSubsystem, false),
            new SequentialCommandGroup(
                new PastXPositionCommand(robotStateSubsystem, driveSubsystem, 3.3),
                new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem))),
        new ParallelCommandGroup(
            new SetVisionUpdateCommand(driveSubsystem, true),
            new ConditionalCommand(
                fallbackPath,
                new AutoPlaceAutonCommand(
                        driveSubsystem,
                        robotStateSubsystem,
                        armSubsystem,
                        handSubsystem,
                        true,
                        DriveConstants.kAutonBumpHighCubeYawOffsetDeg)
                    .withTimeout(0.8),
                () -> !visionSubsystem.isCameraWorking())),
        new ParallelCommandGroup(
            new SetVisionUpdateCommand(driveSubsystem, false),
            new ShootGamepieceCommand(handSubsystem, robotStateSubsystem, true)),
        // Cube 2
        new ParallelDeadlineGroup(
            thirdPath,
            new AutoFloorIntakeCommand(
                robotStateSubsystem, intakeSubsystem, armSubsystem, handSubsystem),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.MID)),
        new ParallelCommandGroup(
            new SetVisionUpdateCommand(driveSubsystem, true),
            new AutoPickupCommand( //  NOT USING JACOBS AUTOPICK IN DRIVESUBSYSTEM
                    driveSubsystem,
                    robotStateSubsystem,
                    RobotStateConstants.kCubeOneAutoPickup,
                    visionSubsystem)
                .withTimeout(0.7)),
        new ParallelDeadlineGroup(
            fourthPath,
            new SetVisionUpdateCommand(driveSubsystem, false),
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CUBE),
            new SequentialCommandGroup(
                new PastXPositionCommand(robotStateSubsystem, driveSubsystem, 2.8),
                new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem))),
        new ParallelCommandGroup(
            new SetVisionUpdateCommand(driveSubsystem, true),
            new ConditionalCommand(
                fallbackPath2,
                new AutoPlaceAutonCommand(
                        driveSubsystem,
                        robotStateSubsystem,
                        armSubsystem,
                        handSubsystem,
                        false,
                        0.0)
                    .withTimeout(0.65),
                () -> !visionSubsystem.isCameraWorking())),
        new ParallelCommandGroup(
            new ShootGamepieceCommand(handSubsystem, robotStateSubsystem, false),
            new ClearGamePieceCommand(robotStateSubsystem))
        /*,new ParallelCommandGroup(
            fourthPath,
            new PastXPositionCommand(
                robotStateSubsystem, driveSubsystem, Constants.AutonConstants.kPastXPosition)),
        new AutoPlaceCommandGroup(driveSubsystem, robotStateSubsystem, armSubsystem, handSubsystem),
        new ClearGamePieceCommand(robotStateSubsystem)*/ );
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
    fourthPath.generateTrajectory();
    fallbackPath.generateTrajectory();
    fallbackPath2.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
