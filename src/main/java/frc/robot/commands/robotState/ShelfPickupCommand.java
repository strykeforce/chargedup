package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ShelfPickupCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;

  public ShelfPickupCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toManualShelf();
  }

  @Override
  public boolean isFinished() {
      return robotStateSubsystem.getRobotState() == RobotState.MANUAL_SHELF;
  }
}
