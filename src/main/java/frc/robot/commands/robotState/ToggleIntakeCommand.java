package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ToggleIntakeCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;

  public ToggleIntakeCommand(
      RobotStateSubsystem robotStateSubsystem, IntakeSubsystem intakeSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    RobotState currState = robotStateSubsystem.getRobotState();
    if (currState == RobotState.INTAKE_STAGE
        || currState == RobotState.PICKUP_FROM_INTAKE
        || currState == RobotState.TO_INTAKE_STAGE) {
      robotStateSubsystem.toStow();
    } else {
      robotStateSubsystem.toIntake();
    }
  }
}
