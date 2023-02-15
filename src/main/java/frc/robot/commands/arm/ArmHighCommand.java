package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHighCommand extends InstantCommand {
  private ArmSubsystem armSubsystem;

  public ArmHighCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    armSubsystem.toHighPos();
  }
}
