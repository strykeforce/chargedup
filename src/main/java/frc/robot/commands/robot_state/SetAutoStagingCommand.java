package frc.robot.commands.robot_state;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotStateSubsystem;

public class SetAutoStagingCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;
  private boolean enabled;

  public SetAutoStagingCommand(RobotStateSubsystem robotStateSubsystem, boolean enable) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.enabled = enable;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setAutoStaging(enabled);
  }
}
