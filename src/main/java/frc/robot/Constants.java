package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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
  public static final double kJoystickDeadband = 0.1;

  public static final class RobotStateConstants {}

  public static final class ArmConstants {
    public static final double kFrontBumperX = 0.25;
    public static final double kCamY = 0.21;
    public static final double kHouseLineSlope = -2.125;
    public static final double kHouseIntercept = 1.06;
    public static final double kIntakeMaxY = 0.37;

    // house limits
    public static final double kShoulderVerticalMin = -1580;
    public static final double kShoulderVerticalMax = -1290;

    public static final double kElevatorHouseMin = -27_295;
    public static final double kElevatorHouseMax = ElevatorConstants.kMaxFwd;

    public static final double kElbowPhysicalMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowPhysicalMax = ElbowConstants.kForwardSoftLimit;

    // bumper limits
    public static final double kShoulderPhysicalMin = ShoulderConstants.kMaxRev;
    public static final double kShoulderPhysicalMax = ShoulderConstants.kMaxFwd;

    public static final double kElevatorBumperMin = -46_258;
    public static final double kElevatorBumperMax = ElevatorConstants.kMaxFwd;

    public static final double kElbowBumperMin = 260;
    public static final double kElbowBumperMax = ElbowConstants.kForwardSoftLimit;

    // intake limits
    // Shoulder Physical Min
    // Shoulder Physical Max

    public static final double kElevatorPhysicalMin = ElevatorConstants.kMaxRev;
    public static final double kElevatorPhysicalMax = ElevatorConstants.kMaxFwd;

    public static final double kElbowIntakeMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowIntakeMax = -450;
  }

  public static final class DriveConstants {
    // Drive Constants
    public static final double kWheelDiameterInches = 3.0 * (575.0 / 500.0); // Actual/Odometry
    public static final double kUpdateThreshold = 0.35;
    public static final double kResetThreshold = 0.005;
    public static final double kPutOdomResetThreshold = 0.35;

    // Drive Base Size and Gearing
    public static final double kMaxSpeedMetersPerSecond = 5.121; // practice bot 3.889
    public static final double kRobotWidth = 0.5; // practice bot: 0.625
    public static final double kRobotLength = 0.615; // practice bot: 0.625

    public static final double kMaxOmega =
        (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0))
            / 2.0; // wheel locations below

    static final double kDriveMotorOutputGear = 30; // practice bot: 22
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
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(0.5, 0.5);
      trajectoryConfig.setReversed(false);
      trajectoryConfig.setStartVelocity(0.0);
      trajectoryConfig.setEndVelocity(0.0);
      return trajectoryConfig;
    }

    // Azimuth Talon Config
    public static SupplyCurrentLimitConfiguration getAzimuthSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 10, 15, 0.04);
    }
  }

  public static final class FieldConstants {
    public static final double kFieldLength = 16.54;
  }

  public static class ElevatorConstants {
    public static final int kLeftMainId = 31;

    public static final double kAllowedError = 100; // FIXME

    public static final double kElevatorZeroSpeed = 0.1;
    public static final double kZeroTargetSpeedTicksPer100ms = 5;
    public static final int kZeroStableCounts = 10; // old 25

    public static final double kMaxFwd = -500;
    public static final double kMaxRev = -49_047;

    public static final double kTicksPerMeter = 62000.0 / 0.4; // FIXME
    public static final double kMaxExtension = 1.23; // FIXME meters

    public static TalonFXConfiguration getElevatorFalconConfig() {
      TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

      elevatorConfig.supplyCurrLimit.currentLimit = 80;
      elevatorConfig.supplyCurrLimit.triggerThresholdCurrent = 90;
      elevatorConfig.supplyCurrLimit.triggerThresholdTime = 0.1;
      elevatorConfig.supplyCurrLimit.enable = true;

      elevatorConfig.statorCurrLimit.currentLimit = 100.0;
      elevatorConfig.statorCurrLimit.triggerThresholdCurrent = 120.0;
      elevatorConfig.statorCurrLimit.triggerThresholdTime = 0.1;
      elevatorConfig.statorCurrLimit.enable = true;

      // elevatorConfig.slot0.kP = 1.0;
      // elevatorConfig.slot0.kI = 0.0;
      // elevatorConfig.slot0.kD = 0.0;
      // elevatorConfig.slot0.kF = 0.065;
      // elevatorConfig.slot0.integralZone = 0;
      // elevatorConfig.slot0.maxIntegralAccumulator = 0;
      // elevatorConfig.slot0.allowableClosedloopError = 0;
      // elevatorConfig.motionCruiseVelocity = 5_000;
      // elevatorConfig.motionAcceleration = 30_000;

      elevatorConfig.forwardSoftLimitEnable = true;
      elevatorConfig.forwardSoftLimitThreshold = kMaxFwd;
      elevatorConfig.reverseSoftLimitEnable = true;
      elevatorConfig.reverseSoftLimitThreshold = kMaxRev;
      elevatorConfig.neutralDeadband = 0.01;
      elevatorConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      elevatorConfig.velocityMeasurementWindow = 64;
      elevatorConfig.voltageCompSaturation = 12;
      elevatorConfig.voltageMeasurementFilter = 32;

      return elevatorConfig;
    }

    public static SupplyCurrentLimitConfiguration getElevatorSupplyLimitConfig() {
      return new SupplyCurrentLimitConfiguration(true, 40, 45, .04);
    }

    public static SupplyCurrentLimitConfiguration getElevatorZeroSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 5, 5, 0.1);
    }
  }

  public static final class ElbowConstants {
    public static final int kElbowFalconID = 33; // 33
    public static final int kRemoteEncoderID = 15; // 15

    // zero=up&slightly towards the elevator
    public static final int kZeroTicks = 1878; // FIXME needs real tick values
    public static final int kForwardSoftLimit = 1905;
    public static final int kReverseSoftLimit = -506;

    public static final int kCloseEnoughTicks = 20;

    public static final double kZeroDegs = -90; // FIXME
    public static final double kTicksPerDeg = 4096.0 / 360.0; // FIXME
    public static final double kLength = 0.7855; // m

    public static TalonFXConfiguration getElbowFalonConfig() {

      TalonFXConfiguration elbowConfig = new TalonFXConfiguration();

      elbowConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 0.5);
      elbowConfig.voltageMeasurementFilter = 32;
      elbowConfig.voltageCompSaturation = 12;
      elbowConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      elbowConfig.velocityMeasurementWindow = 64;
      elbowConfig.neutralDeadband = 0.01;

      return elbowConfig;
    }
  }

  public static final double kCanConfigTimeout = 10; // ms

  public static final class ShoulderConstants {
    public static final int kShoulderId = 30; // FIXME
    public static final int kZeroId = 0; // FIXME

    public static final double kShoulderZeroTicks = 1990; // FIXME old: 1836

    public static final double kMaxFwd = 887; // FIXME
    public static final double kMaxRev = -1580; // FIXME

    public static final double kZeroDegs = 0; // FIXME

    public static final double kTicksPerDeg = 35.55556; // FIXME old: 70.0 / 20.0 * 4096.0 / 360.0

    public static final double kAllowedError = 100; // FIXME

    public static final double kShoulderLen = 0.21; // a
    public static final double kShoulderLowerToElevatorLowerPivotDist = 0.245; // d
    public static final double kShoulderUpperToElevatorUpperPivotDist = 0.1975; // b
    public static final double kElevatorPivotDist = 0.342; // c (old 0.27)
    public static final double kElevatorBaseToPivot = 0.03; // f (old 0.065)
    public static final double kElevatorBaseToElevatorUpperPivot = 0.3325; // g (old 0.23)

    public static TalonSRXConfiguration getShoulderTalonConfig() {
      TalonSRXConfiguration shoulderConfig = new TalonSRXConfiguration();

      // shoulderConfig.slot0.kP = 0.0;
      // shoulderConfig.slot0.kI = 0.0;
      // shoulderConfig.slot0.kD = 0.0;
      // shoulderConfig.slot0.kF = 0.0;
      // shoulderConfig.slot0.integralZone = 0;
      // shoulderConfig.slot0.maxIntegralAccumulator = 0;
      // shoulderConfig.slot0.allowableClosedloopError = 0;
      // shoulderConfig.motionCruiseVelocity = 0;
      // shoulderConfig.motionAcceleration = 0;

      shoulderConfig.forwardSoftLimitEnable = true;
      shoulderConfig.forwardSoftLimitThreshold = kMaxFwd;
      shoulderConfig.reverseSoftLimitEnable = true;
      shoulderConfig.reverseSoftLimitThreshold = kMaxRev;
      shoulderConfig.neutralDeadband = 0.01;
      shoulderConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      shoulderConfig.velocityMeasurementWindow = 64;
      shoulderConfig.voltageCompSaturation = 12;
      shoulderConfig.voltageMeasurementFilter = 32;

      return shoulderConfig;
    }

    public static SupplyCurrentLimitConfiguration getShoulderTalonSupplyLimitConfig() {
      SupplyCurrentLimitConfiguration shoulderSupplyConfig = new SupplyCurrentLimitConfiguration();

      shoulderSupplyConfig.currentLimit = 40;
      shoulderSupplyConfig.triggerThresholdCurrent = 45;
      shoulderSupplyConfig.triggerThresholdTime = .04;
      shoulderSupplyConfig.enable = true;

      return shoulderSupplyConfig;
    }
  }
}
