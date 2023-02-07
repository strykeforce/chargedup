package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {
  PhotonCamera cam1 = new PhotonCamera("OV9281");
  private static final Logger logger = LoggerFactory.getLogger(VisionSubsystem.class);
  public static DriveSubsystem driveSubsystem;
  private CircularBuffer gyroBuffer;
  private CircularBuffer timestampBuffer;
  private boolean canFillBuffers = false;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget bestTarget;
  PhotonPipelineResult result;
  double timeStamp;
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator photonPoseEstimator;
  private boolean buffersFull = false;
  Translation2d robotPose = new Translation2d(2767, 2767);

  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    try {
      aprilTagFieldLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory().toPath() + "/2023-chargedup.json");
    } catch (IOException e) {
      logger.error("VISION SUBSYSTEM : APRILTAG JSON FAILED");
    }
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            cam1,
            // -.25, .11,0 // 0,10, 187
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))));
  }

  public Translation2d cameraOffset() {
    return new Translation2d(
        VisionConstants.kCameraOffset
            * Math.cos(
                Units.degreesToRadians(
                    VisionConstants.kCameraAngleOffset
                        - driveSubsystem.getGyroRotation2d().getDegrees())),
        -VisionConstants.kCameraOffset
            * Math.sin(
                Units.degreesToRadians(
                    VisionConstants.kCameraAngleOffset
                        - driveSubsystem.getGyroRotation2d().getDegrees())));
  }

  public double targetDist(int ID) {
    for (PhotonTrackedTarget temp : targets) {
      if (temp.getFiducialId() == ID) {
        return temp.getBestCameraToTarget().getX();
      }
    }
    return 2767.0;
  }

  public double getBestTarget() {
    if (result.hasTargets()) {
      return bestTarget.getFiducialId();
    }
    return 2767.0;
  }

  public double getNumTargets() {
    return targets.size();
  }

  public double getAmbiguity() {
    if (getHasTargets() == 1.0) {
      return bestTarget.getPoseAmbiguity();
    }
    return 2767;
  }

  public double getHasTargets() {
    if (result.hasTargets()) return 1.0;
    return 0.0;
  }

  public Translation2d getPositionFromRobot() {
    if (result.hasTargets()) {
      double x =
          bestTarget.getBestCameraToTarget().getX(); // + Constants.VisionConstants.kXCameraOffset;
      double y =
          bestTarget.getBestCameraToTarget().getY(); // + Constants.VisionConstants.kYCameraOffset;
      return new Translation2d(x, y); // z is from the camera height
    }
    return new Translation2d(2767, 2767);
  }

  public Translation2d getOdometry() {
    return robotPose;
  }

  public TimestampedPose odomNewPoseViaVision() {
    // List<PhotonTrackedTarget> gyroBuffer;
    if (buffersFull) {
      Rotation2d gyroAngle =
          new Rotation2d(
              gyroBuffer.get(
                  VisionConstants.kBufferLookupOffset)); // driveSubsystem.getGyroRotation2d();
      // List<PhotonTrackedTarget> timestampBuffer;
      double timestamp = timestampBuffer.get(VisionConstants.kBufferLookupOffset);

      Pose2d odomPose = new Pose2d(robotPose, gyroAngle);
      return new TimestampedPose((long) timestamp, odomPose);
    } else {
      return new TimestampedPose((long) 2767, new Pose2d());
    }
  }

  public void fillBuffers() {
    gyroBuffer.addFirst(driveSubsystem.getGyroRotation2d().getRadians());
    timestampBuffer.addFirst(RobotController.getFPGATime());
    buffersFull = true;
  }

  public void setFillBuffers(boolean set) {
    canFillBuffers = set;
  }

  @Override
  public void periodic() {
    try {
      result = cam1.getLatestResult();
      if (canFillBuffers) fillBuffers();
    } catch (Exception e) {
      // logger.info("VISION : GET LATEST FAILED");
    }
    if (result.hasTargets()) {
      targets = result.getTargets();
      bestTarget = result.getBestTarget();
      timeStamp = result.getTimestampSeconds();
    }
    double x = robotPose.getX(), y = robotPose.getY();
    try {
      if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() <= 0.15) {
        x = photonPoseEstimator.update().get().estimatedPose.getX();
        y = photonPoseEstimator.update().get().estimatedPose.getY();
        if (driveSubsystem.distanceOdometryVision(
                new Pose2d(new Translation2d(x, y), new Rotation2d()))
            <= DriveConstants.kUpdateThreshold)
          driveSubsystem.updateOdometryWithVision(
              new Pose2d(new Translation2d(x, y).plus(cameraOffset()), new Rotation2d()),
              (long) timeStamp);
      }
    } catch (Exception e) {
      // logger.error("VISION : ODOMETRY FAIL");
    }
    robotPose = new Translation2d(x, y);
    // result.setTimestampSeconds(timeStamp);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Distance to 1", () -> targetDist(1)),
        new Measure("Distance to 2", () -> targetDist(2)),
        new Measure("Distance to 3", () -> targetDist(3)),
        new Measure("Distance to 4", () -> targetDist(4)),
        new Measure("Distance to 5", () -> targetDist(5)),
        new Measure("Distance to 6", () -> targetDist(6)),
        new Measure("Distance to 7", () -> targetDist(7)),
        new Measure("Distance to 8", () -> targetDist(8)),
        new Measure("Best Target Id", () -> getBestTarget()),
        new Measure("Num Targets", () -> getNumTargets()),
        new Measure("BestTarget Ambiguity", () -> getAmbiguity()),
        new Measure("Time Stamp", () -> timeStamp),
        new Measure(
            "Vision Odometry Y(Offset)", () -> (getOdometry().getX() + cameraOffset().getX())),
        new Measure(
            "Vision Odometry X(Offset)", () -> (getOdometry().getY() + cameraOffset().getY())),
        new Measure("Has Targets", () -> getHasTargets()),
        new Measure("Camera Offset X", () -> cameraOffset().getX()),
        new Measure("Camera Offset Y", () -> cameraOffset().getY()),
        new Measure("Camera Odometry X (NO OFFSET)", () -> getOdometry().getX()),
        new Measure("Camera Odometry Y (NO OFFSET)", () -> getOdometry().getY()));
  }
}
