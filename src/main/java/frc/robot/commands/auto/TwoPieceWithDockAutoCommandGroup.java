package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.hand.ToggleHandCommand;
import frc.robot.commands.robotState.FloorIntakeCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

public class TwoPieceWithDockAutoCommandGroup extends SequentialCommandGroup {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;

  public TwoPieceWithDockAutoCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      IntakeSubsystem intakeSubsystem,
      String pathOne,
      String pathTwo,
      String pathThree) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
    thirdPath = new DriveAutonCommand(driveSubsystem, pathThree, true, false);

    addCommands(
        new SetGamePieceCommand(robotStateSubsystem, GamePiece.CONE),
        new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH),
        new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem),
        new ToggleHandCommand(handSubsystem, robotStateSubsystem, armSubsystem),
        new ParallelCommandGroup(
            firstPath, new FloorIntakeCommand(robotStateSubsystem, armSubsystem, intakeSubsystem)),
        new ParallelCommandGroup(
            secondPath,
            new SequentialCommandGroup(
                new PastXPositionCommand(driveSubsystem, Constants.AutonConstants.kPastXPosition),
                new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem))),
        new ToggleHandCommand(handSubsystem, robotStateSubsystem, armSubsystem),
        thirdPath);
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    thirdPath.generateTrajectory();
  }
}
