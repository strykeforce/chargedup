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
  public static final double kJoystickDeadband = 0.1;

  public static final class RobotStateConstants {}

  public static final class ArmConstants {
    public static final double kFrontBumperX = 0.30; // old: 0.25
    public static final double kCamY = 0.26; // Old: 0.21
    public static final double kHouseLineSlope = -2.125;
    public static final double kHouseIntercept = 1.06;
    public static final double kIntakeMaxY = 0.35; // 0.45
    public static final double kHouseMinX = -0.10; // -0.3
    public static final double kIntakeX = -0.35;
    public static final double kElevatorToElbowPivot = 0.075;
    // public static final double kIntakeY = 0.32;

    // house limits
    public static final double kShoulderVerticalMin = -1580;
    public static final double kShoulderVerticalMax = -1200; // old: -1290

    public static final double kElevatorHouseMin = -10_306;
    public static final double kElevatorHouseMax = ElevatorConstants.kMaxFwd;

    public static final double kElbowPhysicalMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowPhysicalMax = ElbowConstants.kForwardSoftLimit;

    // bumper limits
    public static final double kShoulderPhysicalMin = ShoulderConstants.kMaxRev;
    public static final double kShoulderPhysicalMax = ShoulderConstants.kMaxFwd;

    public static final double kElevatorBumperMin = -40_000; // -46_258
    public static final double kElevatorBumperMax = ElevatorConstants.kMaxFwd;

    public static final double kElbowBumperMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowBumperMax = ElbowConstants.kForwardSoftLimit;

    // intake limits
    // Shoulder Physical Min
    // Shoulder Physical Max

    public static final double kElevatorPhysicalMin = ElevatorConstants.kMaxRev;
    public static final double kElevatorPhysicalMax = ElevatorConstants.kMaxFwd;

    // public static final double kElevatorBelowIntakeMax = -41300;

    public static final double kInsideIntakeElevatorMax = ElevatorConstants.kMaxFwd;
    public static final double kInsideIntakeElevatorMin = -15_585;

    public static final double kElbowInsideIntakeMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowInsideIntakeMax = -7644;
    public static final double kElbowAboveIntakeMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowIntakeMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowIntakeMax = ElbowConstants.kForwardSoftLimit;

    public static final double kShelfMove = 1; // FIXME put in real number
  }

  public static final class DriveConstants {
    // Drive Constants
    public static final Pose2d kOdometryZeroPos =
        new Pose2d(new Translation2d(1.77, 5.12), new Rotation2d());
    public static final double kWheelDiameterInches =
        3.0 * (563.5 / 500.0); // Actual/Odometry //575 old number
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

    public static final double kCameraOffset = .273;
    public static final double kCameraAngleOffset = 24; // DEGREES
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
        VecBuilder.fill(.25, .25, Units.degreesToRadians(5));
  }

  public static final class FieldConstants {
    public static final double kFieldLength = 16.54;
  }

  public static class ElevatorConstants {
    public static final int kLeftMainId = 31;
    public static final int kRightFollowId = 32;

    public static final double kAllowedError = 100; // FIXME

    public static final double kElevatorZeroSpeed = 0.1;
    public static final double kZeroTargetSpeedTicksPer100ms = 5;
    public static final int kZeroStableCounts = 10; // old 25

    public static final double kMaxFwd = -1000; // -500
    public static final double kMaxRev = -25_000; // -49_325

    public static final double kTicksPerMeter = 125_424.3; // 62000.0 / 0.4 meters
    public static final double kMaxExtension = 1.23; // FIXME meters

    // Elevator Positions
    public static final double kIntakeElevator = -13_500;
    public static final double kStowElevator = -2_000;
    public static final double kFloorElevator = -24_065;
    public static final double kLevelOneElevator = -1_500;
    public static final double kLevelTwoElevator = -11_674;
    public static final double kLevelThreeElevator = -1_500;
    public static final double kShelfElevator = -26_106;

    public static TalonFXConfiguration getElevatorFalconConfig() {
      TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

      elevatorConfig.supplyCurrLimit.currentLimit = 80;
      elevatorConfig.supplyCurrLimit.triggerThresholdCurrent = 90;
      elevatorConfig.supplyCurrLimit.triggerThresholdTime = 0.1;
      elevatorConfig.supplyCurrLimit.enable = true;

      elevatorConfig.slot0.kP = 0.3;
      elevatorConfig.slot0.kI = 0.0;
      elevatorConfig.slot0.kD = 0.0;
      elevatorConfig.slot0.kF = 0.047;
      elevatorConfig.slot0.integralZone = 0;
      elevatorConfig.slot0.maxIntegralAccumulator = 0;
      elevatorConfig.slot0.allowableClosedloopError = 0;
      elevatorConfig.motionCruiseVelocity = 5_000;
      elevatorConfig.motionAcceleration = 20_000;

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
      return new SupplyCurrentLimitConfiguration(true, 20, 20, .1);
    }

    public static SupplyCurrentLimitConfiguration getElevatorZeroSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 5, 5, 0.1);
    }
  }

  public static final class ElbowConstants {
    public static final int kElbowFalconID = 33; // 33
    public static final int kRemoteEncoderID = 15; // 15

    // zero=up&slightly towards the elevator
    public static final int kZeroTicks = 1820; // FIXME needs real tick values

    public static final int kForwardSoftLimit = 100_000; // 1905
    public static final int kReverseSoftLimit = -13_375; // -506

    public static final double kZeroDegs = -90; // FIXME
    public static final double kTicksPerDeg = 4096.0 / 360.0; // FIXME
    public static final double kLength = 1; // 0.7855 m

    public static final double kOffsetFactor = 103.5 / 2;

    public static final int kCloseEnoughTicks = 20;

    // Elbow Positions
    public static final double kIntakeElbow = -8_137;
    public static final double kStowElbow = 3_100;
    public static final double kFloorElbow = 20_726;
    public static final double kLevelOneElbow = 24_375;
    public static final double kLevelTwoElbow = 58_581;
    public static final double kLevelThreeElbow = 78_346;
    public static final double kShelfElbow = 58_581;

    public static TalonFXConfiguration getElbowFalonConfig() {

      TalonFXConfiguration elbowConfig = new TalonFXConfiguration();

      elbowConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 5, 5, 0.1);
      elbowConfig.voltageMeasurementFilter = 32;
      elbowConfig.voltageCompSaturation = 12;
      elbowConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      elbowConfig.velocityMeasurementWindow = 64;
      elbowConfig.neutralDeadband = 0.01;

      elbowConfig.slot0.kP = 0.8;
      elbowConfig.slot0.kI = 0.0;
      elbowConfig.slot0.kD = 5.0;
      elbowConfig.slot0.kF = 0.055;
      elbowConfig.slot0.integralZone = 0.0;
      elbowConfig.slot0.maxIntegralAccumulator = 0.0;
      elbowConfig.slot0.allowableClosedloopError = 100.0;

      elbowConfig.motionAcceleration = 6_000;
      elbowConfig.motionCruiseVelocity = 5_000;
      elbowConfig.forwardSoftLimitEnable = true;
      elbowConfig.forwardSoftLimitThreshold = kForwardSoftLimit;
      elbowConfig.reverseSoftLimitEnable = true;
      elbowConfig.reverseSoftLimitThreshold = kReverseSoftLimit;

      return elbowConfig;
    }
  }

  public static final double kCanConfigTimeout = 10; // ms

  public static final class ShoulderConstants {
    public static final int kShoulderId = 30; // FIXME

    public static final double kShoulderZeroTicks = 1990; // FIXME old: 1836

    public static final double kMaxFwd = 500; // FIXME 887
    public static final double kMaxRev = -1700; // FIXME -1580

    public static final double kZeroDegs = 0; // FIXME

    public static final double kTicksPerDeg = 35.55556; // FIXME old: 70.0 / 20.0 * 4096.0 / 360.0

    public static final double kAllowedError = 100; // FIXME

    public static final double kShoulderLen = 0.20; // a 0.21
    public static final double kShoulderLowerToElevatorLowerPivotDist = 0.242; // d 0.245
    public static final double kShoulderUpperToElevatorUpperPivotDist = 0.1975; // b
    public static final double kElevatorPivotDist = 0.342; // c (old 0.27)
    public static final double kElevatorBaseToPivot = 0.05; // f (old 0.065)
    public static final double kElevatorBaseToElevatorUpperPivot = 0.35; // g (old 0.23

    public static final double kOffsetDegs =
        Math.toDegrees(Math.asin(0.06 / kShoulderLowerToElevatorLowerPivotDist));

    // Shoulder Positions
    public static final double kIntakeShoulder = -1_700;
    public static final double kStowShoulder = -1_700;
    public static final double kFloorShoulder = 300;
    public static final double kLevelOneShoulder = 300;
    public static final double kLevelTwoShoulder = -700;
    public static final double kLevelThreeShoulder = 300;
    public static final double kShelfShoulder = -1_950;

    public static TalonSRXConfiguration getShoulderTalonConfig() {
      TalonSRXConfiguration shoulderConfig = new TalonSRXConfiguration();

      shoulderConfig.slot0.kP = 2.0;
      shoulderConfig.slot0.kI = 0.0;
      shoulderConfig.slot0.kD = 60.0;
      shoulderConfig.slot0.kF = 2.0;
      shoulderConfig.slot0.integralZone = 0;
      shoulderConfig.slot0.maxIntegralAccumulator = 0;
      shoulderConfig.slot0.allowableClosedloopError = 0;
      shoulderConfig.motionCruiseVelocity = 200.0;
      shoulderConfig.motionAcceleration = 200.0;

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

      shoulderSupplyConfig.currentLimit = 5;
      shoulderSupplyConfig.triggerThresholdCurrent = 5;
      shoulderSupplyConfig.triggerThresholdTime = .1;
      shoulderSupplyConfig.enable = true;

      return shoulderSupplyConfig;
    }
  }

  public static final class IntakeConstants {

    public static final int kIntakeFalconID = 20;
    public static final int kExtendTalonID = 21;

    public static final int kCloseEnoughTicks = 150;
    public static final int kExtendPosTicks = -1_800;
    public static final int kRetractPosTicks = -200;

    public static final double kIntakeSpeed = -0.35;
    public static final double kIntakeEjectSpeed = 0.3;
    public static final double kEjectTimerDelaySec = 3;
    public static final double kIntakePickupDelaySec = 1;

    public static final int kIntakeZeroTicks = 3955;

    public static TalonSRXConfiguration getExtendTalonConfig() {
      TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();

      talonConfig.forwardSoftLimitEnable = true;
      talonConfig.forwardSoftLimitThreshold = 0;
      talonConfig.reverseSoftLimitEnable = true;
      talonConfig.reverseSoftLimitThreshold = -2500;
      talonConfig.neutralDeadband = 0.01;
      talonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      talonConfig.velocityMeasurementWindow = 64;
      talonConfig.voltageCompSaturation = 12;
      talonConfig.voltageMeasurementFilter = 32;

      talonConfig.slot0.kP = 1.2;
      talonConfig.slot0.kI = 0.0;
      talonConfig.slot0.kD = 20;
      talonConfig.slot0.kF = 1.2;
      talonConfig.slot0.integralZone = 0.0;
      talonConfig.slot0.maxIntegralAccumulator = 0.0;
      talonConfig.slot0.allowableClosedloopError = 0.0;
      talonConfig.motionCruiseVelocity = 400;
      talonConfig.motionAcceleration = 10_000;

      return talonConfig;
    }

    public static SupplyCurrentLimitConfiguration getTalonSupplyLimitConfig() {
      SupplyCurrentLimitConfiguration extendSupplyConfig = new SupplyCurrentLimitConfiguration();

      extendSupplyConfig.currentLimit = 5.0;
      extendSupplyConfig.triggerThresholdCurrent = 20.0;
      extendSupplyConfig.triggerThresholdTime = 1.0;
      extendSupplyConfig.enable = true;

      return extendSupplyConfig;
    }

    public static TalonFXConfiguration getIntakeFalconConfig() {
      TalonFXConfiguration falconConfig = new TalonFXConfiguration();

      falconConfig.neutralDeadband = 0.04;
      falconConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      falconConfig.velocityMeasurementWindow = 64;
      falconConfig.voltageCompSaturation = 12;
      falconConfig.voltageMeasurementFilter = 32;
      falconConfig.supplyCurrLimit.currentLimit = 20;
      falconConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
      falconConfig.supplyCurrLimit.triggerThresholdTime = 0.04;
      falconConfig.supplyCurrLimit.enable = false;

      falconConfig.statorCurrLimit.currentLimit = 30;
      falconConfig.statorCurrLimit.triggerThresholdCurrent = 30;
      falconConfig.statorCurrLimit.triggerThresholdTime = 0.1;
      falconConfig.statorCurrLimit.enable = true;

      return falconConfig;
    }
  }

  public static class HandConstants {
    public static int kHandTalonId = 40;

    public static final double kMaxFwd = 2312; // 1100
    public static final double kMaxRev = -740; // -1000

    public static final double kHasPieceMinTicks = 1000;

    public static final double kHandZeroSpeed = 0.1;
    public static final double kZeroTargetSpeedTicksPer100ms = 5;
    public static final int kZeroStableCounts = 1592;
    public static final int kHasPieceStableCounts = 2;

    public static final double kHandZeroTicks = 855;

    public static final double kAllowedError = 0; // FIXME

    public static final double kHandOpenPosition = -740;
    public static final double kCubeGrabbingPosition = 800; // FIXME
    public static final double kConeGrabbingPosition = 1_500; // FIXME

    public static TalonSRXConfiguration getHandTalonConfig() {
      TalonSRXConfiguration handConfig = new TalonSRXConfiguration();

      handConfig.slot0.kP = 0.0;
      handConfig.slot0.kI = 0.0;
      handConfig.slot0.kD = 0.0;
      handConfig.slot0.kF = 0.85;
      handConfig.slot0.integralZone = 0;
      handConfig.slot0.maxIntegralAccumulator = 0;
      handConfig.slot0.allowableClosedloopError = 0;
      handConfig.motionCruiseVelocity = 500;
      handConfig.motionAcceleration = 5_000;

      handConfig.forwardSoftLimitEnable = true;
      handConfig.forwardSoftLimitThreshold = kMaxFwd;
      handConfig.reverseSoftLimitEnable = true;
      handConfig.reverseSoftLimitThreshold = kMaxRev;
      handConfig.neutralDeadband = 0.04;
      handConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      handConfig.velocityMeasurementWindow = 64;
      handConfig.voltageCompSaturation = 12;
      handConfig.voltageMeasurementFilter = 32;

      return handConfig;
    }

    public static SupplyCurrentLimitConfiguration getHandSupplyLimitConfig() {
      return new SupplyCurrentLimitConfiguration(true, 10, 20, .5);
    }

    public static SupplyCurrentLimitConfiguration getHandZeroSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 5, 5, 0.1);
    }
  }
}
