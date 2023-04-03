package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class ResetOdometryVisionCommand extends InstantCommand {
  public final VisionSubsystem visionSubsystem;
  private Pose2d nearbyPose;

  public ResetOdometryVisionCommand(VisionSubsystem visionSubsystem, Pose2d nearbyPose) {
    this.visionSubsystem = visionSubsystem;
    this.nearbyPose = nearbyPose;
  }

  @Override
  public void initialize() {
    if (visionSubsystem.isCameraWorking()) visionSubsystem.resetOdometry(nearbyPose);
  }
}
