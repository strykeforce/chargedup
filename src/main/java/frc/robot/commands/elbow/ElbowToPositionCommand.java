package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElbowSubsystem;

public class ElbowToPositionCommand extends CommandBase {
  private ElbowSubsystem elbowSubsystem;
  private double position;

  public ElbowToPositionCommand(ElbowSubsystem elbowSubsystem, double position) {
    this.elbowSubsystem = elbowSubsystem;
    this.position = position;

    addRequirements(elbowSubsystem);
  }

  @Override
  public void initialize() {
    elbowSubsystem.setPos(position);
  }

  @Override
  public boolean isFinished() {
    return elbowSubsystem.isElbowAtPos();
  }
}
