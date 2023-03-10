package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ManualScoreCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;

  public ManualScoreCommand(
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem) {
    addRequirements(armSubsystem, handSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toManualStage();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.STOW
        || robotStateSubsystem.getRobotState() == RobotState.MANUAL_SCORE
        || robotStateSubsystem.getRobotState() == RobotState.MANUAL_SHELF;
  }
}
