package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HandSubsystem.HandStates;
import okhttp3.Handshake;

public class GrabConeCommand extends CommandBase {
  private HandSubsystem handSubsystem;

  public GrabConeCommand(HandSubsystem handSubsystem) {
    this.handSubsystem = handSubsystem;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.grabCone();
  }

  @Override
  public boolean isFinished() {
      return handSubsystem.getHandState() == HandStates.CONE_CLOSED;
  }
}
