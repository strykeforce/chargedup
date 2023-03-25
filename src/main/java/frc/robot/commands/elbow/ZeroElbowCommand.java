package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElbowSubsystem;

public class ZeroElbowCommand extends InstantCommand {
  private ElbowSubsystem elbowSubsystem;

  public ZeroElbowCommand(ElbowSubsystem elbowSubsystem) {
    this.elbowSubsystem = elbowSubsystem;
  }

  @Override
  public void initialize() {
    elbowSubsystem.zeroElbowStow();
  }
}
