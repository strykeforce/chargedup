package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSpeedCommand extends InstantCommand {
  private ElevatorSubsystem elevatorSubsystem;
  private double pctSpeed;

  public ElevatorSpeedCommand(ElevatorSubsystem elevatorSubsystem, double pctSpeed) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.pctSpeed = pctSpeed;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setPct(pctSpeed);
  }
}
