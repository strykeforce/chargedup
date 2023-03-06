package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;

public class ArmHighCommand extends CommandBase {
  private ArmSubsystem armSubsystem;
  private GamePiece gamePiece;

  public ArmHighCommand(ArmSubsystem armSubsystem, GamePiece gamePiece) {
    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
    this.gamePiece = gamePiece;
  }

  @Override
  public void initialize() {
    armSubsystem.toHighPos(gamePiece);
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.getCurrState() == ArmState.HIGH_CONE
        || armSubsystem.getCurrState() == ArmState.HIGH_CUBE;
  }
}
