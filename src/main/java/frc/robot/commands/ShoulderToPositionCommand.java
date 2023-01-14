package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderToPositionCommand extends InstantCommand {
  private double position;
  private ShoulderSubsystem shoulderSubsystem;

  public ShoulderToPositionCommand(ShoulderSubsystem shoulderSubsystem, double position) {
    this.shoulderSubsystem = shoulderSubsystem;
    this.position = position;

    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    shoulderSubsystem.setPos(position);
  }
}
