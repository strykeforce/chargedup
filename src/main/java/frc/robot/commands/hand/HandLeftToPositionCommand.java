package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;

public class HandLeftToPositionCommand extends InstantCommand {
  private HandSubsystem handSubsystem;
  private double position;

  public HandLeftToPositionCommand(HandSubsystem handSubsystem, double position) {
    this.handSubsystem = handSubsystem;
    this.position = position;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.setLeftPos(position);
  }
}
