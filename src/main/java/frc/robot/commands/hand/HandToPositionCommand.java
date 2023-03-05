package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;

public class HandToPositionCommand extends CommandBase {
  private HandSubsystem handSubsystem;
  private double leftPosition;
  private double rightPosition;

  public HandToPositionCommand(
      HandSubsystem handSubsystem, double leftPosition /*, double rightPosition*/) {
    this.handSubsystem = handSubsystem;
    this.leftPosition = leftPosition;
    this.rightPosition = rightPosition;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.stowHand(leftPosition /*, rightPosition*/);
  }

  @Override
  public boolean isFinished() {
      return handSubsystem.isFinished();
  }
}
