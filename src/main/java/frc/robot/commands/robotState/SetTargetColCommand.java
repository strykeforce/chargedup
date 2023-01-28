package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;

public class SetTargetColCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;
  private TargetCol targetCol;

  public SetTargetColCommand(RobotStateSubsystem robotStateSubsystem, TargetCol targetCol) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.targetCol = targetCol;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setTargetCol(targetCol);
  }
}
