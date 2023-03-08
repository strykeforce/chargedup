package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class GrabGamepieceCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;
  private GamePiece gamePiece;
  private HandSubsystem handSubsystem;

  public GrabGamepieceCommand(
      HandSubsystem handSubsystem, RobotStateSubsystem robotStateSubsystem, GamePiece gamePiece) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.gamePiece = gamePiece;
    this.handSubsystem = handSubsystem;
    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toGrabGamepiece(gamePiece);
  }

  @Override
  public boolean isFinished() {
    RobotState currState = robotStateSubsystem.getRobotState();
    return handSubsystem.isFinished()
        && (currState == RobotState.GRAB_GAME_PIECE
            || currState == RobotState.TO_STOW
            || currState == RobotState.STOW);
  }
}
