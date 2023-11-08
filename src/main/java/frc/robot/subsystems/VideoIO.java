package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VideoIO {
  @AutoLog
  public static class VideoIOInputs {
    public Pose2d curPose2d = new Pose2d();
  }

  public default void logOdometry(VideoIOInputs curPose2d) {}
}
