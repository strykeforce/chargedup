package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElbowSubsystem;

public class ElbowOpenLoopCommand extends InstantCommand {
  private ElbowSubsystem elbowSubsystem;
  private double percentOutput;

  public ElbowOpenLoopCommand(ElbowSubsystem elbowSubsystem, double percentOutput) {
    addRequirements(elbowSubsystem);

    this.elbowSubsystem = elbowSubsystem;
    this.percentOutput = percentOutput;
  }

  @Override
  public void initialize() {
    elbowSubsystem.rotateOpenLoop(percentOutput);
  }
}
