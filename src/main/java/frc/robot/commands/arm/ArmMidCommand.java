package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;

public class ArmMidCommand extends InstantCommand {
  private ArmSubsystem armSubsystem;
  private GamePiece gamePiece;

  public ArmMidCommand(ArmSubsystem armSubsystem, GamePiece gamePiece) {
    this.armSubsystem = armSubsystem;
    this.gamePiece = gamePiece;
  }

  @Override
  public void initialize() {
    armSubsystem.toMidPos(gamePiece);
  }
}
