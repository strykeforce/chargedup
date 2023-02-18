package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotStateSubsystem;

public class ShelfPickupCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;

  public ShelfPickupCommand(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toMaunalStage();
  }
}
