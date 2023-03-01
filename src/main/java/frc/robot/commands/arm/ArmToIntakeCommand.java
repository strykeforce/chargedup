package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToIntakeCommand extends InstantCommand {
  private ArmSubsystem armSubsystem;

  public ArmToIntakeCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.toIntakePos();
  }
}
