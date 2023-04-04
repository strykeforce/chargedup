package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ResetOdometryVisionCommand extends InstantCommand {
  public final VisionSubsystem visionSubsystem;
  private Pose2d nearbyPose;
  private final RobotStateSubsystem robotStateSubsystem;

  public ResetOdometryVisionCommand(
      VisionSubsystem visionSubsystem, Pose2d nearbyPose, RobotStateSubsystem robotStateSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.nearbyPose = nearbyPose;
  }

  @Override
  public void initialize() {
    if (visionSubsystem.isCameraWorking())
      visionSubsystem.resetOdometry(nearbyPose, robotStateSubsystem.isBlueAlliance());
  }
}
