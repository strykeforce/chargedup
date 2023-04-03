package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class LowerServoCommand extends InstantCommand {
  VisionSubsystem visionSubsystem;

  public LowerServoCommand(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    visionSubsystem.lowerServo();
  }
}
