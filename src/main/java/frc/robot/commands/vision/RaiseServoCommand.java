package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class RaiseServoCommand extends InstantCommand {
  VisionSubsystem visionSubsystem;

  public RaiseServoCommand(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    visionSubsystem.raiseServo();
  }
}
