package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class AdjustElevatorCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private double decrementTicks;

  public AdjustElevatorCommand(ElevatorSubsystem elevatorSubsystem, double decrementTicks) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.decrementTicks = decrementTicks;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    elevatorSubsystem.setPos(elevatorSubsystem.getPos() - decrementTicks);
  }
}
