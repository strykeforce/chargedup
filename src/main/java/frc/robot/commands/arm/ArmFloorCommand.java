package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFloorCommand extends InstantCommand {
  private ArmSubsystem armSubsystem;

  public ArmFloorCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    armSubsystem.toFloorPos();
  }
}
