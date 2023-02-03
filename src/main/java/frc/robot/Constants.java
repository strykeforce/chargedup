package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

  public static final int kTalonConfigTimeout = 10; // ms
  public static final double kDeadeyePowerCycleTimeout = 5; // s

  public static final class DriveConstants {
    // Drive Constants
    public static final double kWheelDiameterInches = 3.0 * (571.0 / 500.0); // Actual/Odometry
    public static final double kUpdateThreshold = 0.35;
    public static final double kResetThreshold = 0.005;
    public static final double kPutOdomResetThreshold = 0.35;

    // From: https://github.com/strykeforce/axis-config/
    public static final double kMaxSpeedMetersPerSecond = 3.889;
    public static final double kRobotWidth = 0.625;
    public static final double kRobotLength = 0.625;

    public static final double kMaxOmega =
        (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0))
            / 2.0; // wheel locations below

    // From: https://github.com/strykeforce/axis-config/
    static final double kDriveMotorOutputGear = 22;
    static final double kDriveInputGear = 48;
    static final double kBevelInputGear = 15;
    static final double kBevelOutputGear = 45;
    public static final double kDriveGearRatio =
        (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

    public static Translation2d[] getWheelLocationMeters() {
      final double x = kRobotLength / 2.0; // front-back, was ROBOT_LENGTH
      final double y = kRobotWidth / 2.0; // left-right, was ROBOT_WIDTH
      Translation2d[] locs = new Translation2d[4];
      locs[0] = new Translation2d(x, y); // left front
      locs[1] = new Translation2d(x, -y); // right front
      locs[2] = new Translation2d(-x, y); // left rear
      locs[3] = new Translation2d(-x, -y); // right rear
      return locs;
    }

    // Teleop Drive Constants
    //    public static final double kDeadbandXLock = 0.2;
    public static final double kDeadbandAllStick = 0.075;
    //    public static final double kCloseEnoughTicks = 10.0;
    public static final double kRateLimitFwdStr = 3.5; // 2
    public static final double kRateLimitYaw = 3; // 3
    //    public static final double kExpoScaleMoveFactor = 0.6; // .6
    // public static final double kRateLimitMove = 0.3;
    public static final double kExpoScaleYawFactor = 0.75;

    public static TalonSRXConfiguration getAzimuthTalonConfig() {
      // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
      TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();

      azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      azimuthConfig.continuousCurrentLimit = 10;
      azimuthConfig.peakCurrentDuration = 0;
      azimuthConfig.peakCurrentLimit = 0;
      azimuthConfig.slot0.kP = 10.0;
      azimuthConfig.slot0.kI = 0.0;
      azimuthConfig.slot0.kD = 100.0;
      azimuthConfig.slot0.kF = 1.0;
      azimuthConfig.slot0.integralZone = 0;
      azimuthConfig.slot0.allowableClosedloopError = 0;
      azimuthConfig.slot0.maxIntegralAccumulator = 0;
      azimuthConfig.motionCruiseVelocity = 800;
      azimuthConfig.motionAcceleration = 10_000;
      azimuthConfig.velocityMeasurementWindow = 64;
      azimuthConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      azimuthConfig.voltageCompSaturation = 12;
      azimuthConfig.voltageMeasurementFilter = 32;
      return azimuthConfig;
    }
    // Drive Falcon Config
    public static TalonFXConfiguration getDriveTalonConfig() {
      TalonFXConfiguration driveConfig = new TalonFXConfiguration();
      driveConfig.supplyCurrLimit.currentLimit = 40;
      driveConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
      driveConfig.supplyCurrLimit.triggerThresholdTime = .04;
      driveConfig.supplyCurrLimit.enable = true;
      driveConfig.statorCurrLimit.enable = false;
      driveConfig.slot0.kP = 0.045;
      driveConfig.slot0.kI = 0.0005;
      driveConfig.slot0.kD = 0.000;
      driveConfig.slot0.kF = 0.047;
      driveConfig.slot0.integralZone = 500;
      driveConfig.slot0.maxIntegralAccumulator = 75_000;
      driveConfig.slot0.allowableClosedloopError = 0;
      driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      driveConfig.velocityMeasurementWindow = 64;
      driveConfig.voltageCompSaturation = 12;
      driveConfig.neutralDeadband = 0.01;
      driveConfig.voltageMeasurementFilter = 32;
      return driveConfig;
    }
    // Holonomic Controller Constants
    public static final double kPHolonomic = 6.0;
    public static final double kIHolonomic = 0.0;
    public static final double kDHolonomic = kPHolonomic / 100.0;

    public static final double kPOmega = 2.5;
    public static final double kIOmega = 0.0;
    public static final double kDOmega = 0.0;
    //    public static final double kMaxVelOmega = kMaxOmega / 2.0;
    public static final double kMaxAccelOmega = 3.14;

    // Default safety path constants
    public static final Pose2d startPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d endPose2d = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));

    public static ArrayList<Translation2d> getDefaultInternalWaypoints() {
      ArrayList<Translation2d> waypoints = new ArrayList<>();
      waypoints.add(new Translation2d(-0.5, 0));
      return waypoints;
    }

    public static TrajectoryConfig getDefaultTrajectoryConfig() {
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1, 1);
      trajectoryConfig.setReversed(true);
      trajectoryConfig.setStartVelocity(0.0);
      trajectoryConfig.setEndVelocity(0.0);
      return trajectoryConfig;
    }

    // Azimuth Talon Config
    public static SupplyCurrentLimitConfiguration getAzimuthSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 10, 15, 0.04);
    }
  }

  public static final class VisionConstants {
    public static final double kApTag1x = 15.514;
    public static final double kApTag2x = 15.514;
    public static final double kApTag3x = 15.514;
    public static final double kApTag4x = 16.179;
    public static final double kApTag5x = 0.362;
    public static final double kApTag6x = 1.027;
    public static final double kApTag7x = 1.027;
    public static final double kApTag8x = 1.027;

    public static final double kApTag1y = 1.072;
    public static final double kApTag2y = 2.748;
    public static final double kApTag3y = 4.424;
    public static final double kApTag4y = 6.750;
    public static final double kApTag5y = 6.750;
    public static final double kApTag6y = 4.424;
    public static final double kApTag7y = 2.748;
    public static final double kApTag8y = 1.072;

    public static final double kYCameraOffset = .09;
    public static final double kXCameraOffset = .28;
    public static int kBufferLookupOffset = 2;

    public static Matrix<N3, N1> kStateStdDevs =
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    // Increase these numbers to trust sensor readings from encoders and gyros less. This matrix is
    // in the form [theta], with units in radians.
    public static Matrix<N1, N1> kLocalMeasurementStdDevs =
        VecBuilder.fill(Units.degreesToRadians(0.01));

    // Increase these numbers to trust global measurements from vision less. This matrix is in the
    // form [x, y, theta]ᵀ, with units in meters and radians.
    // Vision Odometry Standard devs
    public static Matrix<N3, N1> kVisionMeasurementStdDevs =
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  }
}
