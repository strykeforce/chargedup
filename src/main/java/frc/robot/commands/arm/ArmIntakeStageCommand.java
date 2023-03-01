package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmIntakeStageCommand extends InstantCommand {
  private ArmSubsystem armSubsystem;

  public ArmIntakeStageCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.toIntakeStagePos();
  }
}
