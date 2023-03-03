package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class ArmLowCommand extends CommandBase {
  private ArmSubsystem armSubsystem;

  public ArmLowCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.toLowPos();
  }

  @Override
  public boolean isFinished() {
      return armSubsystem.getCurrState() == ArmState.LOW;
  }
}
