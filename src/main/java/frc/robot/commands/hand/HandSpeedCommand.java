package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;

public class HandSpeedCommand extends InstantCommand {
  private HandSubsystem handSubsystem;
  private double leftPct;
  private double rightPct;

  public HandSpeedCommand(HandSubsystem handSubsystem, double leftPct, double rightPct) {
    this.handSubsystem = handSubsystem;
    this.leftPct = leftPct;
    this.rightPct = rightPct;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.setLeftPct(leftPct);
    // handSubsystem.setRightPct(rightPct);
  }
}
