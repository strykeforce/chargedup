package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class toggleHealthBoolean extends InstantCommand {
  ArmSubsystem armSubsystem;

  public toggleHealthBoolean(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    armSubsystem.toggleHealthBoolean();
  }
}
