package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;

public class GrabCubeCommand extends InstantCommand {
  private HandSubsystem handSubsystem;

  public GrabCubeCommand(HandSubsystem handSubsystem) {
    this.handSubsystem = handSubsystem;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.grabCube();
  }
}
