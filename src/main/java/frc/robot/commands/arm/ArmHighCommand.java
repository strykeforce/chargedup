package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;

public class ArmHighCommand extends InstantCommand {
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
}
