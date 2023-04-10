package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ShootGamepieceCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;
  private HandSubsystem handSubsystem;
  private boolean doStow = false;

  public ShootGamepieceCommand(
      HandSubsystem handSubsystem, RobotStateSubsystem robotStateSubsystem, boolean doStow) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.handSubsystem = handSubsystem;
    this.doStow = doStow;
    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toShootCube(doStow);
  }

  @Override
  public boolean isFinished() {
    RobotState currState = robotStateSubsystem.getRobotState();
    return handSubsystem.isFinished()
        && (currState == RobotState.RELEASE_GAME_PIECE
            || currState == RobotState.TO_STOW
            || currState == RobotState.STOW);
  }
}
