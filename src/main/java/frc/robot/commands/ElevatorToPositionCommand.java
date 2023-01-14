package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToPositionCommand extends InstantCommand {
  private ElevatorSubsystem elevatorSubsystem;
  private double position;

  public ElevatorToPositionCommand(ElevatorSubsystem elevatorSubsystem, double position) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = position;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setPos(position);
  }
}
