package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {
  PhotonCamera cam1 = new PhotonCamera("cam1");
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget bestTarget;
  PhotonPipelineResult result;
  double timeStamp;
  List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  AprilTagFieldLayout aprilTagFieldLayout;
  RobotPoseEstimator robotPoseEstimator;

  public VisionSubsystem() {
    camList.add(
        new Pair<PhotonCamera, Transform3d>(
            cam1, new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0))));
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    robotPoseEstimator =
        new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);
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
    Translation2d position = getPositionFromRobot();
    // switch ((int) getBestTarget()) {
    //   case 1:
    //     return new Translation2d(
    //         position.getX() + Constants.VisionConstants.kApTag1x,
    //         position.getY() - Constants.VisionConstants.kApTag1y);
    //   case 2:
    //     return new Translation2d(
    //         position.getX() + Constants.VisionConstants.kApTag2x,
    //         position.getY() - Constants.VisionConstants.kApTag2y);
    //   case 3:
    //     return new Translation2d(
    //         position.getX() + Constants.VisionConstants.kApTag3x,
    //         position.getY() - Constants.VisionConstants.kApTag3y);
    //   case 4:
    //     return new Translation2d(
    //         position.getX() + Constants.VisionConstants.kApTag4x,
    //         position.getY() - Constants.VisionConstants.kApTag4y);
    //   case 5:
    //     return new Translation2d(
    //         position.getX() + Constants.VisionConstants.kApTag5x,
    //         position.getY() - Constants.VisionConstants.kApTag5y);
    //   case 6:
    //     return new Translation2d(
    //         position.getX() + Constants.VisionConstants.kApTag6x,
    //         Constants.VisionConstants.kApTag6y - position.getY());
    //   case 7:
    //     return new Translation2d(
    //         position.getX() + Constants.VisionConstants.kApTag7x,
    //         Constants.VisionConstants.kApTag7y - position.getY());
    //   case 8:
    //     return new Translation2d(
    //         position.getX() + Constants.VisionConstants.kApTag8x,
    //         Constants.VisionConstants.kApTag8y - position.getY());
    //   default:
    //     return new Translation2d(2767, 2767);
    // }
    return new Translation2d(
        robotPoseEstimator.update().get().getFirst().getX(),
        robotPoseEstimator.update().get().getFirst().getY());
  }

  @Override
  public void periodic() {
    result = cam1.getLatestResult();
    targets = result.getTargets();
    bestTarget = result.getBestTarget();
    timeStamp = result.getTimestampSeconds();
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
        new Measure("Position From Robot X", () -> getOdometry().getX()),
        new Measure("Position From Robot Y", () -> getOdometry().getY()),
        new Measure("Has Targets", () -> getHasTargets()),
        new Measure("Supposed x pos", () -> getOdometry().getX()),
        new Measure("Supposed y pos", () -> getOdometry().getY()));
  }
}
