package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;

public class HandRightToPositionCommand extends InstantCommand {
  private HandSubsystem handSubsystem;
  private double position;

  public HandRightToPositionCommand(HandSubsystem handSubsystem, double position) {
    this.handSubsystem = handSubsystem;
    this.position = position;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.setRightPos(position);
  }
}
