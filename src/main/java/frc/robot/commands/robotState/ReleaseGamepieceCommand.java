package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ReleaseGamepieceCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;
  private HandSubsystem handSubsystem;

  public ReleaseGamepieceCommand(
      HandSubsystem handSubsystem, RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.handSubsystem = handSubsystem;
    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toReleaseGamepiece();
  }

  @Override
  public boolean isFinished() {
    RobotState currState = robotStateSubsystem.getRobotState();
    return handSubsystem.isFinished()
        && (currState == RobotState.RELEASE_GAME_PIECE
            || currState == RobotState.TO_STOW_SCORE
            || currState == RobotState.TO_STOW
            || currState == RobotState.STOW);
  }
}
