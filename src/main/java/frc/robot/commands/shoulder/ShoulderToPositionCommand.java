package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderToPositionCommand extends CommandBase {
  private double position;
  private ShoulderSubsystem shoulderSubsystem;

  public ShoulderToPositionCommand(ShoulderSubsystem shoulderSubsystem, double position) {
    this.shoulderSubsystem = shoulderSubsystem;
    this.position = position;

    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    shoulderSubsystem.setPos(position);
  }

  @Override
  public boolean isFinished() {
    return shoulderSubsystem.isFinished();
  }
}
