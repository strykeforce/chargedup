package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class HoldPositionCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public HoldPositionCommand(ElevatorSubsystem elevatorSubsystem, RobotStateSubsystem robotStateSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.getRobotState() != RobotState.RELEASE_GAME_PIECE) { 
      elevatorSubsystem.setPos(elevatorSubsystem.getPos());
    }
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isFinished();
  }
}
