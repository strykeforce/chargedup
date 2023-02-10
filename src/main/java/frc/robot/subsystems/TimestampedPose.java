package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class TimestampedPose {
  private long timestamp;
  private Pose2d pose;

  public TimestampedPose(long timestamp, Pose2d pose) {
    this.timestamp = timestamp;
    this.pose = pose;
  }

  public long getTimestamp() {
    return timestamp;
  }

  public void setTimestamp(long newTime) {
    this.timestamp = newTime;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setPose(Pose2d newPose) {
    this.pose = newPose;
  }
}
