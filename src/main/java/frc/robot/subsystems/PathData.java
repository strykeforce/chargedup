package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class PathData {
  public final Rotation2d targetYaw;
  public final Trajectory trajectory;

  public PathData(Rotation2d targetYaw, Trajectory trajectory) {
    this.targetYaw = targetYaw;
    this.trajectory = trajectory;
  }
}
