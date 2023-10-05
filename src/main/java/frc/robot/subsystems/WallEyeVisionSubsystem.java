package frc.robot.subsystems;

import WallEye.WallEye;
import WallEye.WallEyeResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Set;
import net.jafama.FastMath;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class WallEyeVisionSubsystem extends MeasurableSubsystem {
  private DigitalInput[] empty = {};

  public static DriveSubsystem driveSubsystem;
  private DigitalInput[] diosHigh = {};
  private DigitalInput[] diosLow = {};
  private Pose2d suppliedCamPose = new Pose2d();
  private boolean trustingWheels = true;
  private VisionStates curState = VisionStates.trustWheels;
  private double timeLastCam = 0.0;
  private int timesCamOffWheel = 0;
  private int numUpdateForReset = 0;

  private WallEye[] cams;
  private String[] names = {"High", "Low"};
  private int[] numCams = {1, 1};
  private DigitalInput[][] dios = {diosHigh, diosLow};
  private WallEyeResult[][] results;
  private Pose3d[] camPoses = {new Pose3d(), new Pose3d()};
  private int[] updates = {0, 0};
  private int[] numTags = {0, 0};
  private double[] camDelay = {0, 0};
  private double[] camAmbig = {0, 0};
  private double[] camLengths = {VisionConstants.kHighCameraOffset, VisionConstants.kCameraOffset};
  private double[] camAngle = {
    VisionConstants.kHighCameraAngleOffset, VisionConstants.kCameraAngleOffset
  };

  public Matrix<N3, N1> adaptiveVisionMatrix;

  public WallEyeVisionSubsystem(DriveSubsystem driveSubsystem) {
    adaptiveVisionMatrix = Constants.VisionConstants.kVisionMeasurementStdDevs.copy();
    this.driveSubsystem = driveSubsystem;

    for (int i = 0; i < names.length; ++i) {
      cams[i] = new WallEye(names[i], numCams[i], dios[i]);
      cams[i].setCamToCenter(0, cameraOffset(camLengths[i], camAngle[i]));
    }
  }

  public Transform3d cameraOffset(double length, double angle) {
    return new Transform3d(
        new Translation3d(
            length
                * FastMath.cos(
                    Units.degreesToRadians(
                        angle - driveSubsystem.getGyroRotation2d().getDegrees())),
            -length
                * FastMath.sin(
                    Units.degreesToRadians(
                        angle - driveSubsystem.getGyroRotation2d().getDegrees())),
            0.0),
        new Rotation3d());
  }

  @Override
  public void periodic() {
    boolean noUpdate = true;
    for (int i = 0; i < names.length; ++i) {
      if (cams[i].hasNewUpdate()) {
        noUpdate = false;
        results[i] = cams[i].getResults();
        camDelay[i] = RobotController.getFPGATime() - results[i][0].getTimeStamp();
        numTags[i] = results[i][0].getNumTags();
        camPoses[i] = cams[i].camPoseToCenter(i, results[i][0].getCameraPose());
        updates[i] = cams[i].getUpdateNumber();
        camAmbig[i] = results[i][0].getAmbiguity();

        handleCameraFilter(results[i], cams[i]);
      }
    }
    if (noUpdate) {
      if (getTimeSec() - timeLastCam > VisionConstants.kTimeToTightenStdDev) {
        if (getTimeSec() - timeLastCam > VisionConstants.kTimeToTrustCamera)
          curState = VisionStates.trustVision;

        numUpdateForReset = 0;
        for (int i = 0; i < 2; ++i) {
          double estimatedWeight =
              VisionConstants.kVisionMeasurementStdDevs.get(i, 0)
                  - VisionConstants.kStdDevDecayCoeff
                      * ((getTimeSec() - timeLastCam) - VisionConstants.kTimeToTightenStdDev);

          adaptiveVisionMatrix.set(
              i,
              0,
              estimatedWeight < VisionConstants.kMinimumStdDev
                  ? VisionConstants.kMinimumStdDev
                  : estimatedWeight);
        }
      }
    }
  }

  private void handleCameraFilter(WallEyeResult[] results, WallEye cam) {
    try {
      for (WallEyeResult res : results) {
        Pose2d camPose = cam.camPoseToCenter(0, res.getCameraPose()).toPose2d();
        if ((res.getAmbiguity() < 0.15 || res.getNumTags() > 1)) {
          timeLastCam = getTimeSec();
          switch (curState) {
            case trustWheels:
              if (canAcceptPose(camPose)) {
                handleVision(res);
                timesCamOffWheel = 0;
              } else {
                timesCamOffWheel++;
              }
              if (numUpdateForReset > VisionConstants.kNumResultsToResetStdDev)
                adaptiveVisionMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();

              break;
            case trustVision:
              handleVision(res);
              if (numUpdateForReset > VisionConstants.kNumResultsToTrustWheels) {
                curState = VisionStates.trustWheels;
                adaptiveVisionMatrix = VisionConstants.kVisionMeasurementStdDevs.copy();
              }
              break;
            case onlyTrustVision:
              break;
            case onlyTrustWheels:
              break;
          }
        }
      }
    } catch (Exception e) {
    }
  }

  public void handleVision(WallEyeResult res) {
    numUpdateForReset++;
    suppliedCamPose = res.getCameraPose().toPose2d();
    driveSubsystem.updateOdometryWithVision(suppliedCamPose, res.getTimeStamp() / 1000000);
  }

  public double getTimeSec() {
    return (double) RobotController.getFPGATime() / 1000000;
  }

  public void switchVisionMode(boolean doTrustWheels) {
    trustingWheels = doTrustWheels;
    curState = trustingWheels ? VisionStates.trustWheels : curState;
  }

  public boolean trustWheels() {
    return trustingWheels;
  }

  private boolean canAcceptPose(Pose2d cam) {
    ChassisSpeeds speed = driveSubsystem.getFieldRelSpeed();
    Pose2d curPose = driveSubsystem.getPoseMeters();
    Transform2d disp = curPose.minus(cam);
    double magnitudeVel =
        Math.sqrt(Math.pow(speed.vxMetersPerSecond, 2) + Math.pow(speed.vyMetersPerSecond, 2));
    double magnitudeDisp = Math.sqrt(Math.pow(disp.getX(), 2) + Math.pow(disp.getY(), 2));

    return (magnitudeDisp
        < ((magnitudeVel * VisionConstants.kLinearCoeffOnVelFilter)
            + VisionConstants.kOffsetOnVelFilter
            + Math.pow((magnitudeVel * VisionConstants.kSquaredCoeffOnVelFilter), 2)));
  }

  public WallEyeResult[] getPoses() {
    return results[0];
  }

  public Pose2d[] getPose2ds() {
    ArrayList<Pose2d> poses = new ArrayList<>();
    for (int i = 0; i < cams.length; ++i) {
      for (WallEyeResult res : results[i]) poses.add(res.getCameraPose().toPose2d());
    }
    return (Pose2d[]) poses.toArray();
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("High Cam x", () -> camPoses[0].getX()),
        new Measure("High Cam y", () -> camPoses[0].getY()),
        new Measure("High Cam z", () -> camPoses[0].getZ()),
        new Measure("High latency", () -> camDelay[0] / 1000),
        new Measure("High Update num", () -> updates[0]),
        new Measure("High Tag Ambig", () -> camAmbig[0]),
        new Measure("High Supplied Camera Pose X", () -> suppliedCamPose.getX()),
        new Measure("High Supplied Camera Pose Y", () -> suppliedCamPose.getY()),
        new Measure("High Vision State", () -> curState.ordinal()),
        new Measure("High X Y standard devs for vision", () -> adaptiveVisionMatrix.get(0, 0)));
  }

  public enum VisionStates {
    trustWheels,
    trustVision,
    onlyTrustVision,
    onlyTrustWheels,
  }
}
