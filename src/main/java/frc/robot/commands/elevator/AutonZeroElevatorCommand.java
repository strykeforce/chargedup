package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

public class AutonZeroElevatorCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;

  public AutonZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.autonZeroElevator();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getElevatorState() == ElevatorState.ZEROED;
  }
}
