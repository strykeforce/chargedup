package frc.robot.commands.robot_state;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

public class SetLevelAndColCommandGroup extends ParallelCommandGroup {
  public SetLevelAndColCommandGroup(
      RobotStateSubsystem robotStateSubsystem, TargetLevel targetLevel, TargetCol targetCol) {
    addCommands(
        new SetTargetLevelCommand(robotStateSubsystem, targetLevel),
        new SetTargetColCommand(robotStateSubsystem, targetCol));
  }
}
