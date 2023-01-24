package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsytems.ElbowSubsystem;

public class ElbowOpenLoopCommand extends CommandBase {
  private ElbowSubsystem elbowSubsystem;
  private double percentOutput;

  public ElbowOpenLoopCommand(ElbowSubsystem elbowSubsystem, double percentOutput) {
    this.elbowSubsystem = elbowSubsystem;
    this.percentOutput = percentOutput;
  }

  @Override
  public void initialize() {
    elbowSubsystem.rotateOpenLoop(percentOutput);
  }
}
