package frc.robot.subsystems;

import WallEye.WallEye;
import WallEye.WallEyeResult;
import ch.qos.logback.core.recovery.ResilientFileOutputStream;
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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Set;
import net.jafama.FastMath;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {
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
  private boolean hasGottenUpdates = false;
  private Servo highCamera; 

  private WallEye[] cams;
  private String[] names = {"High", "Low"};
  private int[] numCams = {1, 1};
  private DigitalInput[][] dios = {diosHigh, diosLow};
  private WallEyeResult[][] results;
  private Pose3d[] camPoses = {new Pose3d(), new Pose3d()};
  private int[] updates = {0,0};
  private int[] numTags = {0, 0};
  private double[] camDelay = {0,0};
  private double[] camAmbig = {0,0};
  private double[] camLengths = {VisionConstants.kHighCameraOffset, VisionConstants.kCameraOffset};
  private double[] camAngle = {VisionConstants.kHighCameraAngleOffset, VisionConstants.kCameraAngleOffset};

  public Matrix<N3, N1> adaptiveVisionMatrix;

  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    highCamera = new Servo(1);
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
                        angle
                            - driveSubsystem.getGyroRotation2d().getDegrees())),
            -length
                * FastMath.sin(
                    Units.degreesToRadians(
                        angle
                            - driveSubsystem.getGyroRotation2d().getDegrees())),
            0.0),
        new Rotation3d());
  }

  @Override
  public void periodic() {
    boolean noUpdate = true;
    for (int i = 0; i < names.length; ++i) {
        if (cams[i].hasNewUpdate()) {
            hasGottenUpdates = true;
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

  public boolean isCameraWorking() {
    return hasGottenUpdates;
  }

  public WallEyeResult[] getPoses() {
    return results[0];
  }

  public void raiseServo() {
    highCamera.set(VisionConstants.kServoUpPos);
  }
    
  public void lowerServo() {
    highCamera.set(VisionConstants.kServoDownPos);
  }


  public Pose2d[] getPose2ds() {
    ArrayList<Pose2d> poses = new ArrayList<>();
    for (int i = 0; i < cams.length; ++i) {
        for (WallEyeResult res : results[i])
            poses.add(res.getCameraPose().toPose2d());
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



// package frc.robot.subsystems;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.util.CircularBuffer;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Servo;
// import frc.robot.Constants.RobotStateConstants;
// import frc.robot.Constants.VisionConstants;
// import java.io.IOException;
// import java.util.List;
// import java.util.Set;
// import net.jafama.FastMath;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;
// import org.strykeforce.telemetry.TelemetryService;
// import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
// import org.strykeforce.telemetry.measurable.Measure;

// public class VisionSubsystem extends MeasurableSubsystem {
//   PhotonCamera cam1 = new PhotonCamera("OV9281");
//   PhotonCamera cam2 = new PhotonCamera("OV9281HIGH");

//   private static final Logger logger = LoggerFactory.getLogger(VisionSubsystem.class);
//   public static DriveSubsystem driveSubsystem;
//   private CircularBuffer gyroBuffer = new CircularBuffer(10000);
//   private CircularBuffer timestampBuffer = new CircularBuffer(10000);
//   private CircularBuffer velocityBuffer = new CircularBuffer(10000);
//   private boolean canFillBuffers = false;
//   List<PhotonTrackedTarget> highTargets;
//   List<PhotonTrackedTarget> targets;
//   PhotonTrackedTarget bestTarget;
//   PhotonPipelineResult result;
//   PhotonPipelineResult resultHighCamera;
//   double timeStamp;
//   double highTimeStamp;
//   AprilTagFieldLayout aprilTagFieldLayout;
//   PhotonPoseEstimator photonPoseEstimator;
//   PhotonPoseEstimator highCameraEstimator;
//   private boolean buffersFull = false;
//   private boolean hasResetOdomAuto = false;
//   private boolean useHigh = false;
//   private EstimatedRobotPose savedOffRobotEstimation;
//   Translation2d robotPose = new Translation2d(2767, 2767);
//   private int gyroBufferId = 0;
//   private int visionOff = 0;
//   private Pose3d lastPose;
//   private int lastUpdate = 0;
//   private int currUpdate = 0;
//   private Pose3d cameraPose = new Pose3d(new Translation3d(2767.0, 2767.0, 0.0), new Rotation3d());
//   private final Notifier photonVisionThread;
//   private Servo highCameraMount;

//   public VisionSubsystem(DriveSubsystem driveSubsystem) {
//     photonVisionThread = new Notifier(this::visionUpdateThread);
//     photonVisionThread.startPeriodic(20.0 / 1000.0);
//     this.driveSubsystem = driveSubsystem;
//     highCameraMount = new Servo(1);
//     try {
//       aprilTagFieldLayout =
//           new AprilTagFieldLayout(
//               Filesystem.getDeployDirectory().toPath() + "/2023-chargedup.json");
//     } catch (IOException e) {
//       logger.error("VISION SUBSYSTEM : APRILTAG JSON FAILED");
//     }
//     highCameraEstimator =
//         new PhotonPoseEstimator(
//             aprilTagFieldLayout,
//             PoseStrategy.MULTI_TAG_PNP,
//             cam2,
//             new Transform3d(
//                 new Translation3d(0, 0, 0),
//                 new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))));
//     photonPoseEstimator =
//         new PhotonPoseEstimator(
//             aprilTagFieldLayout,
//             PoseStrategy.LOWEST_AMBIGUITY,
//             cam1,
//             // -.25, .11,0 // 0,10, 187
//             new Transform3d(
//                 new Translation3d(0, 0, 0),
//                 new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))));
//   }

//   public Translation2d cameraOffset() {
//     return new Translation2d(
//         VisionConstants.kCameraOffset
//             * FastMath.cos(
//                 Units.degreesToRadians(
//                     VisionConstants.kCameraAngleOffset
//                         - driveSubsystem.getGyroRotation2d().getDegrees())),
//         -VisionConstants.kCameraOffset
//             * FastMath.sin(
//                 Units.degreesToRadians(
//                     VisionConstants.kCameraAngleOffset
//                         - driveSubsystem.getGyroRotation2d().getDegrees())));
//   }

//   public Translation2d highCameraOffset() {
//     return new Translation2d(
//         VisionConstants.kHighCameraOffset
//             * FastMath.cos(
//                 Units.degreesToRadians(
//                     VisionConstants.kHighCameraAngleOffset
//                         - driveSubsystem.getGyroRotation2d().getDegrees())),
//         -VisionConstants.kHighCameraAngleOffset
//             * FastMath.sin(
//                 Units.degreesToRadians(
//                     VisionConstants.kHighCameraAngleOffset
//                         - driveSubsystem.getGyroRotation2d().getDegrees())));
//   }

//   public double targetDist(int ID) {
//     for (PhotonTrackedTarget temp : targets) {
//       if (temp.getFiducialId() == ID) {
//         return temp.getBestCameraToTarget().getX();
//       }
//     }
//     return 2767.0;
//   }

//   public void raiseServo() {
//     highCameraMount.set(VisionConstants.kServoUpPos);
//   }

//   public void lowerServo() {
//     highCameraMount.set(VisionConstants.kServoDownPos);
//   }

//   public double getBestTarget() {
//     if (result.hasTargets()) {
//       return bestTarget.getFiducialId();
//     }
//     return 2767.0;
//   }

//   public double getNumTargets() {
//     return targets.size();
//   }

//   public double getAmbiguity() {
//     if (!isCameraWorking()) return 2767;
//     if (getHasTargets() == 1.0) {
//       return bestTarget.getPoseAmbiguity();
//     }
//     return 2767;
//   }

//   public double getHasTargets() {
//     if (result.hasTargets()) return 1.0;
//     return 0.0;
//   }

//   public Translation2d getPositionFromRobot() {
//     if (result.hasTargets()) {
//       double x =
//           bestTarget.getBestCameraToTarget().getX(); // + Constants.VisionConstants.kXCameraOffset;
//       double y =
//           bestTarget.getBestCameraToTarget().getY(); // + Constants.VisionConstants.kYCameraOffset;
//       return new Translation2d(x, y); // z is from the camera height
//     }
//     return new Translation2d(2767, 2767);
//   }

//   public Translation2d getOdometry() {
//     return robotPose;
//   }

//   public TimestampedPose odomNewPoseViaVision() {
//     // List<PhotonTrackedTarget> gyroBuffer;
//     if (buffersFull) {
//       Rotation2d gyroAngle =
//           new Rotation2d(
//               gyroBuffer.get(
//                   VisionConstants.kBufferLookupOffset)); // driveSubsystem.getGyroRotation2d();
//       // List<PhotonTrackedTarget> timestampBuffer;
//       double timestamp = timestampBuffer.get(VisionConstants.kBufferLookupOffset);

//       Pose2d odomPose = new Pose2d(robotPose, gyroAngle);
//       return new TimestampedPose((long) timestamp, odomPose);
//     } else {
//       return new TimestampedPose((long) 2767, new Pose2d());
//     }
//   }

//   public void fillBuffers() {
//     gyroBuffer.addFirst(driveSubsystem.getGyroRotation2d().getRadians());
//     timestampBuffer.addFirst(RobotController.getFPGATime());
//     velocityBuffer.addFirst(driveSubsystem.getVectorSpeed());
//     buffersFull = true;
//   }

//   public void setFillBuffers(boolean set) {
//     logger.info("Set Fill Buffers: {}", set);
//     canFillBuffers = set;
//   }

//   public boolean isCameraWorking() {
//     return cam1.isConnected() && cam2.isConnected();
//   }

//   public double getBufferedVelocity() {
//     return velocityBuffer.get(gyroBufferId);
//   }

//   public void resetOdometry(Pose2d errorCheckPose, boolean isBlueAlliance) {
//     if (!isBlueAlliance)
//       errorCheckPose =
//           new Pose2d(
//               new Translation2d(
//                   RobotStateConstants.kFieldMaxX - errorCheckPose.getX(), errorCheckPose.getY()),
//               new Rotation2d());

//     if (cam2.isConnected()
//         && FastMath.abs(errorCheckPose.getY() - cameraPose.getY()) <= 1.0 // 0.75
//         && ((isBlueAlliance && cameraPose.getX() <= 6.9 && cameraPose.getX() > 5.1)
//             || (!isBlueAlliance
//                 && cameraPose.getX() >= RobotStateConstants.kFieldMaxX - 6.9
//                 && cameraPose.getX() < RobotStateConstants.kFieldMaxX - 5.1))) {
//       logger.info("reseting to X : {} | Y : {}", cameraPose.getX(), cameraPose.getY());
//       driveSubsystem.resetOdometry(
//           new Pose2d(
//               new Translation2d(cameraPose.getX(), cameraPose.getY()),
//               driveSubsystem.getGyroRotation2d()));
//     } else {
//       // double tempX = (driveSubsystem.getPoseMeters().getX() + cameraPose.getX()) / 2;
//       double tempY = (driveSubsystem.getPoseMeters().getY() + cameraPose.getY()) / 2;
//       // logger.info("TempX: {}, TempY: {}", tempX, tempY);
//       Pose2d tempPose =
//           new Pose2d(
//               new Translation2d(driveSubsystem.getPoseMeters().getX(), tempY),
//               driveSubsystem.getGyroRotation2d());
//       logger.info(
//           "Averaging Odom, error: {} | Camera: {}, Resetting to pose: {}, drive Odom: {}",
//           FastMath.hypot(
//               (cameraPose.getX() - errorCheckPose.getX()),
//               (cameraPose.getY() - errorCheckPose.getY())),
//           cameraPose,
//           tempPose,
//           driveSubsystem.getPoseMeters());
//       driveSubsystem.resetOdometry(tempPose);
//     }
//   }

//   @Override
//   public void periodic() {
//     if (driveSubsystem.canGetVisionUpdates() && lastUpdate < currUpdate) {
//       lastUpdate = currUpdate;
//       double y = cameraPose.getY();
//       double x = cameraPose.getX();
//       Pose2d temp = new Pose2d(new Translation2d(x, y), new Rotation2d());
//       if (((FastMath.hypot(
//                       x - driveSubsystem.getPoseMeters().getX(),
//                       y - driveSubsystem.getPoseMeters().getY())
//                   <= 0.75
//               || (visionOff > 0
//                   && FastMath.hypot(x - lastPose.getX(), y - lastPose.getY()) <= 0.75)))
//           && !(useHigh && x >= 13 && x <= 3)) {
//         visionOff = 0;
//         driveSubsystem.updateOdometryWithVision(
//             new Pose2d(
//                 new Translation2d(x, y).plus(useHigh ? highCameraOffset() : cameraOffset()),
//                 new Rotation2d(gyroBuffer.get(gyroBufferId))),
//             (long) (useHigh ? highTimeStamp : timeStamp));

//         if (driveSubsystem.canGetVisionUpdates() && driveSubsystem.isAutoDriving()) {
//           driveSubsystem.resetOdometryNoLog( // FIXME
//               new Pose2d(
//                   new Translation2d(x, y).plus(useHigh ? highCameraOffset() : cameraOffset()),
//                   new Rotation2d(gyroBuffer.get(gyroBufferId))));
//           setOdomAutoBool(true);
//           // result.setTimestampSeconds(timeStamp);
//         }
//       } else {
//         logger.info("Bad reading");
//         visionOff++;
//         lastPose = cameraPose;
//       }
//       robotPose = new Translation2d(x, y);
//     }
//   }

//   public boolean getOdomAutoBool() {
//     return hasResetOdomAuto;
//   }

//   public double getLastUpdateTime() {
//     return timeStamp;
//   }

//   public boolean lastUpdateWithinThresholdTime(double threshold) {
//     return (RobotController.getFPGATime() / 1000000) - timeStamp <= threshold;
//   }

//   public void setOdomAutoBool(boolean autoBool) {
//     logger.info("setOdomAutoBool: {}", autoBool);
//     hasResetOdomAuto = autoBool;
//   }

//   private void visionUpdateThread() {
//     try {
//       result = cam1.getLatestResult();
//       if (canFillBuffers) fillBuffers();
//       // logger.info("Filled Gyro Buffer: {}", canFillBuffers);
//     } catch (Exception e) {
//     }
//     try {
//       resultHighCamera = cam2.getLatestResult();
//     } catch (Exception e) {
//     }
//     // logger.info("VISION : GET LATEST FAILED");
//     if (result.hasTargets()) {
//       targets = result.getTargets();
//       bestTarget = result.getBestTarget();
//       timeStamp = result.getTimestampSeconds();
//       gyroBufferId = (int) FastMath.floor(result.getLatencyMillis() / 20);
//     }
//     if (resultHighCamera.hasTargets()) {
//       highTargets = resultHighCamera.getTargets();
//       highTimeStamp = resultHighCamera.getTimestampSeconds();
//     }
//     if (result.hasTargets()
//         && (result.getBestTarget().getPoseAmbiguity() <= 0.15 || result.targets.size() > 1)
//         && !(resultHighCamera.hasTargets()
//             && resultHighCamera.getTargets().size() < result.getTargets().size())) {
//       try {
//         savedOffRobotEstimation = photonPoseEstimator.update().get();
//         cameraPose = savedOffRobotEstimation.estimatedPose;
//         useHigh = false;
//         currUpdate++;
//       } catch (Exception e) {
//       }
//     } else {
//       if (resultHighCamera.hasTargets()
//           && (resultHighCamera.getBestTarget().getPoseAmbiguity() <= 0.15
//               || resultHighCamera.targets.size() > 1)) {
//         try {
//           savedOffRobotEstimation = highCameraEstimator.update().get();
//           useHigh = true;
//           cameraPose = savedOffRobotEstimation.estimatedPose;
//           currUpdate++;
//         } catch (Exception e) {
//         }
//       }
//     }
//   }

//   @Override
//   public void registerWith(TelemetryService telemetryService) {
//     super.registerWith(telemetryService);
//   }

//   @Override
//   public Set<Measure> getMeasures() {
//     return Set.of(
//         new Measure("Distance to 1", () -> targetDist(1)),
//         new Measure("Distance to 2", () -> targetDist(2)),
//         new Measure("Distance to 3", () -> targetDist(3)),
//         new Measure("Distance to 4", () -> targetDist(4)),
//         new Measure("Distance to 5", () -> targetDist(5)),
//         new Measure("Distance to 6", () -> targetDist(6)),
//         new Measure("Distance to 7", () -> targetDist(7)),
//         new Measure("Distance to 8", () -> targetDist(8)),
//         new Measure("Best Target Id", () -> getBestTarget()),
//         new Measure("Num Targets", () -> getNumTargets()),
//         new Measure("BestTarget Ambiguity", () -> getAmbiguity()),
//         new Measure("Time Stamp", () -> timeStamp),
//         new Measure(
//             "Vision Odometry X(Offset)", () -> (getOdometry().getX() + cameraOffset().getX())),
//         new Measure(
//             "Vision Odometry Y(Offset)", () -> (getOdometry().getY() + cameraOffset().getY())),
//         new Measure(
//             "Vision Odometry X(HighOffset)",
//             () -> (getOdometry().getX() + highCameraOffset().getX())),
//         new Measure(
//             "Vision Odometry Y(HighOffset)",
//             () -> (getOdometry().getY() + highCameraOffset().getY())),
//         new Measure("Has Targets", () -> getHasTargets()),
//         new Measure("Camera Offset X", () -> cameraOffset().getX()),
//         new Measure("Camera Offset Y", () -> cameraOffset().getY()),
//         new Measure("Camera Latency", () -> result.getLatencyMillis()),
//         new Measure("Camera Odometry X NO", () -> getOdometry().getX()),
//         new Measure("hasResetOdomAuto", () -> (getOdomAutoBool() ? 1 : 0)),
//         new Measure("gyro bufferId", () -> gyroBufferId),
//         new Measure("Gyro Buffer", () -> new Rotation2d(gyroBuffer.get(gyroBufferId)).getDegrees()),
//         new Measure("canFillBuffers", () -> canFillBuffers ? 1 : 0),
//         new Measure("Camera Odometry Y NO", () -> getOdometry().getY()),
//         new Measure(
//             "Num targets",
//             () ->
//                 savedOffRobotEstimation == null
//                     ? 2767.0
//                     : savedOffRobotEstimation.targetsUsed.size()),
//         new Measure("Is using High", () -> useHigh ? 1.0 : 0.0),
//         new Measure("Camera Pose X", () -> cameraPose.getX()),
//         new Measure("Camera Pose Y", () -> cameraPose.getY()));
//   }
// }
