package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class ResetOdometryVisionCommand extends InstantCommand {
  public final VisionSubsystem visionSubsystem;

  public ResetOdometryVisionCommand(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    if (visionSubsystem.isCameraWorking()) visionSubsystem.resetOdometry();
  }
}
