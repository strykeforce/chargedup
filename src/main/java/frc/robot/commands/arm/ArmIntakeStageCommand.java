package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class ArmIntakeStageCommand extends CommandBase {
  private ArmSubsystem armSubsystem;

  public ArmIntakeStageCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.toIntakeStagePos();
  }

  @Override
  public boolean isFinished() {
      return armSubsystem.getCurrState() == ArmState.INTAKE_STAGE;
  }
}
