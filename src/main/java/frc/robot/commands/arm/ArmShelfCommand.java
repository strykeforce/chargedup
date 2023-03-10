package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class ArmShelfCommand extends CommandBase {
  private ArmSubsystem armSubsystem;

  public ArmShelfCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.toShelfPos();
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.getCurrState() == ArmState.SHELF;
  }
}
