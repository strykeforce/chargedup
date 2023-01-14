package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderSpeedCommand extends InstantCommand {
  private double pctVelocity;
  private ShoulderSubsystem shoulderSubsystem;

  public ShoulderSpeedCommand(ShoulderSubsystem shoulderSubsystem, double pctVelocity) {
    this.shoulderSubsystem = shoulderSubsystem;
    this.pctVelocity = pctVelocity;

    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    shoulderSubsystem.setPct(pctVelocity);
  }
}
