package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class HoldPositionCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;

  public HoldPositionCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setPos(elevatorSubsystem.getPos());
  }

  @Override
  public boolean isFinished() {
      return elevatorSubsystem.isFinished();
  }
}
