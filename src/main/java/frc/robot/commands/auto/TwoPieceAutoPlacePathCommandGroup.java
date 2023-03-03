package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmHighCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.hand.ToggleHandCommand;
import frc.robot.commands.robotState.AutoPlaceCommandGroup;
import frc.robot.commands.robotState.AutonFloorIntakeCommand;
import frc.robot.commands.robotState.GrabConeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;

public class TwoPieceAutoPlacePathCommandGroup extends SequentialCommandGroup {
  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;

  public TwoPieceAutoPlacePathCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      IntakeSubsystem intakeSubsystem,
      String pathOne,
      String pathTwo) {
    addCommands(
        new GrabConeCommand(handSubsystem, robotStateSubsystem, armSubsystem),
        new ArmHighCommand(armSubsystem, GamePiece.CONE),
        new ToggleHandCommand(handSubsystem, robotStateSubsystem, armSubsystem),
        new ParallelCommandGroup(
            new DriveAutonCommand(driveSubsystem, pathOne, true, true),
            new AutonFloorIntakeCommand(robotStateSubsystem, armSubsystem, intakeSubsystem)),
        new ParallelCommandGroup(
            new DriveAutonCommand(driveSubsystem, pathTwo, true, false),
            new ArmHighCommand(armSubsystem, GamePiece.CUBE)),
        new AutoPlaceCommandGroup(driveSubsystem, robotStateSubsystem, armSubsystem, handSubsystem),
        new ToggleHandCommand(handSubsystem, robotStateSubsystem, armSubsystem));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
  }
}
