package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

public class SetTargetLevelCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;
  private TargetLevel targetLevel;

  public SetTargetLevelCommand(RobotStateSubsystem robotStateSubsystem, TargetLevel targetLevel) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.targetLevel = targetLevel;

    addRequirements(robotStateSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setTargetLevel(targetLevel);
  }
}
