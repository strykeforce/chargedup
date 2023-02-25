package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;

public class FloorIntakeCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;

  public FloorIntakeCommand(
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(armSubsystem);
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toIntake();
  }
}
