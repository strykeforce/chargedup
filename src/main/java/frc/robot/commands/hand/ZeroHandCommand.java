package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;

public class ZeroHandCommand extends InstantCommand {
  private HandSubsystem handSubsystem;

  public ZeroHandCommand(HandSubsystem handSubsystem) {
    this.handSubsystem = handSubsystem;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.zeroHand();
  }
}
