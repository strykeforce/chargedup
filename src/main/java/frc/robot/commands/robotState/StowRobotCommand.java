package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class StowRobotCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;

  public StowRobotCommand(
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem,
      HandSubsystem handSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(armSubsystem);
    addRequirements(intakeSubsystem);
    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toStow();
  }

  @Override
  public boolean isFinished() {
      return robotStateSubsystem.getRobotState() == RobotState.STOW;
  }
}
