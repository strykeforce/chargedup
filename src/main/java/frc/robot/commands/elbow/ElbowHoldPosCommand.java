package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElbowSubsystem;

public class ElbowHoldPosCommand extends CommandBase {
  private ElbowSubsystem elbowSubsystem;

  public ElbowHoldPosCommand(ElbowSubsystem elbowSubsystem) {
    addRequirements(elbowSubsystem);
    this.elbowSubsystem = elbowSubsystem;
  }

  @Override
  public void initialize() {
    elbowSubsystem.setPos(elbowSubsystem.getPos());
  }

  @Override
  public boolean isFinished() {
    return elbowSubsystem.isFinished();
  }
}
