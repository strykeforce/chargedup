package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

public class ZeroElevatorCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;

  public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.zeroElevator();
  }

  @Override
  public boolean isFinished() {
      return elevatorSubsystem.getElevatorState() == ElevatorState.ZEROED;
  }
}
