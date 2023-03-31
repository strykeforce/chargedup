package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class AdjustElevatorCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private double decrementTicks;

  public AdjustElevatorCommand(ElevatorSubsystem elevatorSubsystem, RobotStateSubsystem robotStateSubsystem, double decrementTicks) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.decrementTicks = decrementTicks;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    if (robotStateSubsystem.getRobotState() != RobotState.RELEASE_GAME_PIECE)
    {
      elevatorSubsystem.setPos(elevatorSubsystem.getPos() - decrementTicks);
    }
  }
}
