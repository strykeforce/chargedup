package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmLowCommand extends InstantCommand {
  private ArmSubsystem armSubsystem;

  public ArmLowCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    armSubsystem.toLowPos();
  }
}
