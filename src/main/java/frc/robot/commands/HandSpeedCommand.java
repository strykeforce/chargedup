package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;

public class HandSpeedCommand extends InstantCommand {
  private HandSubsystem handSubsystem;
  private double pct;

  public HandSpeedCommand(HandSubsystem handSubsystem, double pct) {
    this.handSubsystem = handSubsystem;
    this.pct = pct;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.setPct(pct);
  }
}
