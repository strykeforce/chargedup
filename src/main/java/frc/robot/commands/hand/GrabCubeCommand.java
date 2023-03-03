package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HandSubsystem.HandStates;

public class GrabCubeCommand extends CommandBase {
  private HandSubsystem handSubsystem;

  public GrabCubeCommand(HandSubsystem handSubsystem) {
    this.handSubsystem = handSubsystem;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.grabCube();
  }

  @Override
  public boolean isFinished() {
      return handSubsystem.getHandState() == HandStates.CUBE_CLOSED;
  }
}
