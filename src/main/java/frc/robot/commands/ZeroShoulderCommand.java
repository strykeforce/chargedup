package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShoulderSubsystem;

public class ZeroShoulderCommand extends InstantCommand {
  private ShoulderSubsystem shoulderSubsystem;

  public ZeroShoulderCommand(ShoulderSubsystem shoulderSubsystem) {
    this.shoulderSubsystem = shoulderSubsystem;

    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    shoulderSubsystem.zeroShoulder();
  }
}
