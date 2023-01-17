package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
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

  public VisionSubsystem() {}

  public double targetDist(int ID) {
    for (PhotonTrackedTarget temp : targets) {
      if (temp.getFiducialId() == ID) {
        return temp.getBestCameraToTarget().getX();
      }
    }
    return 2767.0;
  }

  public double getBestTarget() {
    return bestTarget.getFiducialId();
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
    double x = bestTarget.getBestCameraToTarget().getX() + Constants.VisionConstants.kXCameraOffset;
    double y = bestTarget.getBestCameraToTarget().getY() + Constants.VisionConstants.kYCameraOffset;
    return new Translation2d(x, y); // z is from the camera height
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
        new Measure("Position From Robot X", () -> getPositionFromRobot().getX()),
        new Measure("Position From Robot Y", () -> getPositionFromRobot().getY()),
        new Measure("Has Targets", () -> getHasTargets()));
  }
}
