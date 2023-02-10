package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;

public class HandRightSpeedCommand extends InstantCommand {
  private HandSubsystem handSubsystem;
  private double pct;

  public HandRightSpeedCommand(HandSubsystem handSubsystem, double pct) {
    this.handSubsystem = handSubsystem;
    this.pct = pct;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.setRightPct(pct);
  }
}
