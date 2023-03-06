package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HandSubsystem.HandStates;

public class ZeroHandCommand extends CommandBase {
  private HandSubsystem handSubsystem;

  public ZeroHandCommand(HandSubsystem handSubsystem) {
    this.handSubsystem = handSubsystem;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.zeroHand();
  }

  @Override
  public boolean isFinished() {
    return handSubsystem.getHandState() == HandStates.ZEROED;
  }
}
