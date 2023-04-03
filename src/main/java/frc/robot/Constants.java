package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
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
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

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
  public static String kProtoSerialNumber = "030dbdd8";
  private Logger logger = LoggerFactory.getLogger(Constants.class);
  public static boolean isCompBot =
      (!RobotController.getSerialNumber().equals(kProtoSerialNumber)); // FIXME
  public static double kWheelDiameterInches = 3.0 * (490 / 500.0);
  public static int kElbowZeroTicks = 730;
  public static double kShoulderMainZeroTicks = 1472; // FIXME old: 1836
  public static double kShoulderFollowerZeroTicks = 3167;
  public static int kIntakeZeroTicks = 3150;
  public static double kHandZeroTicks = 975;
  public static double kExtendPosTicks = -2_100;

  public Constants() {
    if (isCompBot) {
      logger.info("Using Comp Robot Constants.");
      kWheelDiameterInches = CompConstants.kWheelDiameterInches;
      kElbowZeroTicks = CompConstants.kElbowZeroTicks;
      kShoulderMainZeroTicks = CompConstants.kShoulderMainZeroTicks;
      kShoulderFollowerZeroTicks = CompConstants.kShoulderFollowerZeroTicks;
      kIntakeZeroTicks = CompConstants.kIntakeZeroTicks;
      kHandZeroTicks = CompConstants.kHandZeroTicks;
      kExtendPosTicks = CompConstants.kExtendPosTicks;
    } else {
      logger.info("Using Proto Robot Constants.");
      kWheelDiameterInches = ProtoConstants.kWheelDiameterInches;
      kElbowZeroTicks = ProtoConstants.kElbowZeroTicks;
      kShoulderMainZeroTicks = ProtoConstants.kShoulderMainZeroTicks;
      kShoulderFollowerZeroTicks = ProtoConstants.kShoulderFollowerZeroTicks;
      kIntakeZeroTicks = ProtoConstants.kIntakeZeroTicks;
      kHandZeroTicks = ProtoConstants.kHandZeroTicks;
      kExtendPosTicks = ProtoConstants.kExtendPosTicks;
    }
  }

  public static final class RobotStateConstants {
    public static final double kRobotLength = 1.5; // FIXME m
    public static final double kPoleToCenterOffset = 1.38 + kRobotLength / 2.0; // m
    public static final double kAutoPlaceX = 1.85; // FIXME m 2.5
    public static final double kRetakeAfterPlaceOffset = 1.0;

    public static final Pose2d kShelfBlue =
        new Pose2d(new Translation2d(15.33, 6.749796), new Rotation2d());
    public static final Pose2d kShelfRed =
        new Pose2d(new Translation2d(1.14, 6.749796), new Rotation2d());
    public static final Pose2d kCubeFourAutoPickup = 
        new Pose2d(new Translation2d(6.9, 4.56), new Rotation2d());
    public static final Pose2d kCubeThreeAutoPickup =
        new Pose2d(new Translation2d(6.9, 3.36), new Rotation2d());
    public static final Pose2d kCubeTwoAutoPickup =
        new Pose2d(new Translation2d(6.9, 2.14), new Rotation2d());
    public static final Pose2d kCubeOneAutoPickup =
        new Pose2d(new Translation2d(6.9, 0.92), new Rotation2d());

    public static final double kPolePlaceOffset = 0.56;
    public static final double kShelfOffset = 0.75;
    public static final double[] kGridY = {1.071626, 2.748026, 4.424426}; // m
    public static final double kBound1Y = 1.908175; // m
    public static final double kBound2Y = 3.584575; // m

    public static final double kReleaseDelayTime = 0.5; // s
    public static final double kFieldMaxX = 16.540988; // m
  }

  public static final class ArmConstants {
    public static final double kFrontBumperX = 0.25; // 0.3
    public static final double kCamY = 0.26; // Old: 0.21

    public static final double kHouseLineSlope = -2.125;
    public static final double kHouseIntercept = 1.06;
    public static final double kIntakeMaxY = 0.35; // 0.45
    public static final double kHouseMinX = -0.10; // -0.3
    public static final double kIntakeX = -0.35;
    public static final double kElevatorToElbowPivot = 0.075;
    // public static final double kIntakeY = 0.32;

    // house limits
    public static final double kShoulderVerticalMin = ShoulderConstants.kMaxRev; // -1580
    public static final double kShoulderVerticalMax = 1200; // -1200

    public static final double kElevatorHouseMin = -23_000; // -10_306
    public static final double kElevatorHouseMax = ElevatorConstants.kMaxFwd;

    public static final double kElbowPhysicalMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowPhysicalMax = ElbowConstants.kForwardSoftLimit;

    // bumper limits
    public static final double kShoulderPhysicalMin = ShoulderConstants.kMaxRev;
    public static final double kShoulderPhysicalMax = ShoulderConstants.kMaxFwd;

    public static final double kElevatorBumperMin = -50_000; // -46_258
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
    public static final double kInsideIntakeElevatorMin = kElevatorPhysicalMin;

    public static final double kElbowInsideIntakeMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowInsideIntakeMax = -9491;
    public static final double kElbowAboveIntakeMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowIntakeMin = ElbowConstants.kReverseSoftLimit;
    public static final double kElbowIntakeMax = ElbowConstants.kForwardSoftLimit;

    public static final double kShelfMove = 0.5; // FIXME put in real number
    public static final double kShelfTransitionMove = 0.06; // 0.2
    public static final double kSweepTimerElapseSeconds = 0.1;
  }

  public static final class AutonConstants {
    public static final double kPastXPosition = 8.0; // FIXME put in real number
    public static final double kMinXFastStow = 2.1;

    public static final int kStartSwitchID = 0;
    public static final int kEndSwitchId = 5;
    public static final int kSwitchStableCounts = 100;
  }

  public static final class DriveConstants {
    // Actual/Odometry //563.5 old number

    public static final double kMaxSpeedMetersPerSecond = 5.44; // practice bot 3.889

    static final double kDriveMotorOutputGear = 30; // practice bot: 22
    static final double kDriveInputGear = 44; // 48
    static final double kBevelInputGear = 15;
    static final double kBevelOutputGear = 45; // 45

    // Drive Constants
    // public static final double kWheelDiameterInches = 3.0 * (563.5 / 500.0); // practice bot
    public static final Pose2d kOdometryZeroPosBlue =
        new Pose2d(new Translation2d(1.80, 5.097), new Rotation2d());
    public static final Pose2d kOdometryZeroPosRed =
        new Pose2d(
            new Translation2d(RobotStateConstants.kFieldMaxX - 1.80, 0.39), new Rotation2d());

    public static final double kShelfMovePercent = 0.8;
    public static final double kShelfYawPercent = 0.2;
    public static final double kPlaceMovePercent = 0.2;
    public static final double kPlaceYawPercent = 0.2;

    // Drive Base Size and Gearing
    public static final double kRobotWidth = 0.495; // practice bot: 0.625 //Old: .5
    public static final double kRobotLength = 0.62; // practice bot: 0.625 //Old:.615

    // Drive Base Size and Gearing

    public static final double kMaxOmega =
        (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0))
            / 2.0; // wheel locations below

    public static final double kDriveGearRatio =
        (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);
    public static double kMaxSpeedToAutoDrive = 1.0; // FIXME WRoNG VAL 1.5
    public static double kPathErrorThreshold = 0.04; // FIXME WRONG VAL 0.03
    public static double kPathErrorOmegaThresholdDegrees = 5; // FIXME WRONG VAL

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

    // AutoBalance Constants

    public static final double kAutoBalanceCloseEnoughDeg = 1.5; // 2
    public static final double kAutoBalanceStableCount = 10;
    public static final double kAutoBalanceStartTimerThresholdDeg = 5;
    public static final double kAutoBalanceEnableGyroThresholdDegrees = 8;

    public static final double kAutoBalanceSlowDriveVel = 0.35; // 0.6
    public static final double kAutoBalanceFinalDriveVel = 1.0; // 0.5 0.75
    public static final double kAutoBalanceSlowdownTimeSec = 1.0; // 1.3-> 1.15
    public static final double kAutoBalanceStopThresholdDegrees = 1.5; // 1 0.6 1.5
    public static final double kAutoBalanceEdgeTriggerThreshold = 3; // 5
    public static final double kAutoBalanceAvgRollCount = 7; // 5 10 7
    public static final double kAutoBalanceLoopFixTimer = 0.140;

    public static final double kPulseAutoBalanceTime = 0.2;
    public static final double kPauseAutoBalanceTime = 1.0;
    public static final double kPulseSpeed = 0.5;
    public static final double kHoldSpeed = 0.1;

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
      azimuthConfig.neutralDeadband = 0.04;
      return azimuthConfig;
    }
    // Drive Falcon Config
    public static TalonFXConfiguration getDriveTalonConfig() {
      TalonFXConfiguration driveConfig = new TalonFXConfiguration();
      driveConfig.supplyCurrLimit.currentLimit = 40;
      driveConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
      driveConfig.supplyCurrLimit.triggerThresholdTime = 1.0;
      driveConfig.supplyCurrLimit.enable = true;
      driveConfig.statorCurrLimit.enable = false;
      driveConfig.slot0.kP = 0.16; // 0.16
      driveConfig.slot0.kI = 0.0002;
      driveConfig.slot0.kD = 0.000;
      driveConfig.slot0.kF = 0.047;
      driveConfig.slot0.integralZone = 500;
      driveConfig.slot0.maxIntegralAccumulator = 150_000;
      driveConfig.slot0.allowableClosedloopError = 0;
      driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      driveConfig.velocityMeasurementWindow = 64;
      driveConfig.voltageCompSaturation = 12;
      driveConfig.neutralDeadband = 0.01;
      driveConfig.voltageMeasurementFilter = 32;
      return driveConfig;
    }
    // Holonomic Controller Constants
    public static final double kPHolonomic = 0.25; // 6 0.25
    public static final double kIHolonomic = 0.0000;
    public static final double kDHolonomic = 0.00; // kPHolonomic/100
    public static final double kIMin = 0.0;
    public static final double kIMax = 0.0;

    public static final double kPOmega = 4.5;
    public static final double kIOmega = 0.0;
    public static final double kDOmega = 0.0;
    //    public static final double kMaxVelOmega = kMaxOmega / 2.0;
    public static final double kMaxAccelOmega = 5.0; // 3.14

    // AUTODRIVE ProfiledPID Constants
    public static final double kPAutoDrive = 3; // 3
    public static final double kIAutoDrive = 0.0000;
    public static final double kDAutoDrive = 0.00; // kPHolonomic/100

    public static final double kAutoDriveMaxVelocity = 2; //
    public static final double kAutoDriveMaxAccel = 2;

    // Auto PickUp Constants
    public static final double kPAutoPickup = 2.5;
    public static final double kIAutoPickup = 0.0;
    public static final double kDAutoPickup = 0.0;

    public static final double kAutoPickupDriveMaxVel = 1.5;
    public static final double kAutoPickupDriveMaxAccel = 2;

    public static final double kAutoPickupCloseEnough = 0.07;

    // Default safety path constants
    public static final Pose2d startPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d endPose2d = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));

    public static final double kAutoDriveAutoYawMax = 30;
    public static final double kMaxSpeedForCamUpdate =
        0.75; // Max Speed(MPS) The robot can be at while the camera updates to autodrive.
    public static final double kPastBumpIndicateX = 12;
    public static final double kArmToAutoDriveDelaySec =
        0; // time delay between arm and autodrive starting

    public static final double kAutoDriveAutoYawCloseEnough = 1;

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

    public static final double kCameraOffset = .335; // was .273 on driveChasis
    public static final double kCameraAngleOffset = 0; // DEGREES was 24 on driveChasis
    public static final double kHighCameraAngleOffset = 0.0;
    public static final double kHighCameraOffset = -0.075;
    public static final double kLastUpdateCloseEnoughThreshold = 2.0; // IN SECONDS
    public static final double kLastUpdateCloseEnoughThresholdYaw = 1.0;
    public static final double kDifferenceCloseEnoughThreshold = .1;
    public static final double kServoId = 1;
    public static final double kServoDownPos = 0.8;
    public static final double kServoUpPos = 0.3;
    public static int kBufferLookupOffset = 2;

    public static Matrix<N3, N1> kStateStdDevs =
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

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

  public static final class FieldConstants {
    public static final double kFieldLength = 16.54;
  }

  public static class ElevatorConstants {
    public static final int kLeftMainId = 31;
    public static final int kRightFollowId = 32;

    public static final double kAllowedError = 1500;
    public static final double kStartNextAxisReinforce = -13_000;

    public static final double kElevatorZeroSpeed = 0.30;
    public static final double kElevatorReinforceSpeed = -0.20;
    public static final double kZeroTargetSpeedTicksPer100ms = 5;
    public static final int kZeroStableCounts = 2; // old 10

    public static final double kMaxFwd = -1000; // -500
    public static final double kMaxRev = -27_281; // -25000

    public static final double kTicksPerMeter = 125_424.3; // 62000.0 / 0.4 meters
    public static final double kMaxExtension = 1.23; // FIXME meters

    // Elevator Positions
    public static final double kIntakeElevator = -23_500; // intake 9.75 gap
    public static final double kStowElevator = -2_000;
    public static final double kFloorElevator = -27_000; // 32648
    public static final double kLevelOneElevator = -10_500;
    public static final double kLevelTwoConeElevator = -23_674; // -23_674 // FIXME
    public static final double kLevelTwoCubeElevator = -40_000; // old -47_674
    public static final double kLevelThreeConeElevator = -1_500; // -1_500 // FIXME
    public static final double kLevelThreeCubeElevator = kLevelThreeConeElevator;
    public static final double kShelfElevator = -2_000; // -18_606
    public static final double kShelfExitElevator = kShelfElevator + 4000; // +3000
    public static final double kShelfMinimumShelfPosition = -43_000;
    public static final double kAutoHighCubeElevator = -2_000;
    public static final double kAutoLevelTwoConeElevator = -23_674;
    public static final double kAutoLevelThreeConeElevator = -1_500;
    public static final double kAutoLevelTwoCubeElevator = -2_000;

    // Parallel Movement Constants
    public static final double kStowToHighElevatorParallelAllowed = kStowElevator;
    public static final double kStowToMidParallelAllowed = kStowElevator;
    public static final double kFloorToStowParallelAllowed = kStowElevator;
    public static final double kScoreToStowParallelAllowed = kStowElevator;
    public static final double kHighToStowParallelAllowed = kStowElevator;
    public static final double kIntakeStageToIntakeParallelAllowed = -3_000; // -10_000
    public static final double kIntakeToStowParallelAllowed = kStowElevator;
    public static final double kShelfToStowParallelAllowed = kStowElevator;
    public static final double kStowToShelfParallelAllowed = kStowElevator;

    public static TalonFXConfiguration getElevatorFalconConfig() {
      TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

      elevatorConfig.supplyCurrLimit.currentLimit = 20;
      elevatorConfig.supplyCurrLimit.triggerThresholdCurrent = 20;
      elevatorConfig.supplyCurrLimit.triggerThresholdTime = 0.1;
      elevatorConfig.supplyCurrLimit.enable = true;

      elevatorConfig.slot0.kP = 0.3;
      elevatorConfig.slot0.kI = 0.0;
      elevatorConfig.slot0.kD = 0.0;
      elevatorConfig.slot0.kF = 0.047;
      elevatorConfig.slot0.integralZone = 0;
      elevatorConfig.slot0.maxIntegralAccumulator = 0;
      elevatorConfig.slot0.allowableClosedloopError = 0;
      elevatorConfig.motionCruiseVelocity = 10_000; // 10_000 -> 12_000
      elevatorConfig.motionAcceleration = 100_000; // 100_000 -> 200_000

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

    public static StatorCurrentLimitConfiguration getElevStatorCurrentLimitConfiguration() {
      return new StatorCurrentLimitConfiguration(true, 8.0, 8.0, 0.001);
    }

    public static StatorCurrentLimitConfiguration getElevStatorTurnOff() {
      return new StatorCurrentLimitConfiguration(false, 0.0, 0.0, 0.0);
    }

    public static SupplyCurrentLimitConfiguration getElevatorSupplyLimitConfig() {
      return new SupplyCurrentLimitConfiguration(true, 20, 20, .1);
    }

    public static SupplyCurrentLimitConfiguration getElevatorZeroSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 1, 1, 0.1);
    }
  }

  public static final class ElbowConstants { // FIXME needs real tick values

    public static final int kElbowFalconID = 33; // 33
    public static final int kRemoteEncoderID = 15; // 15

    // zero=up&slightly towards the elevator

    public static final int kForwardSoftLimit = 90_211; // 187_094
    public static final int kReverseSoftLimit = -22_624; // -506

    public static final double kZeroDegs = -90; // FIXME
    public static final double kTicksPerDeg = 4096.0 / 360; // FIXME
    public static final double kLength = 0.9; // 0.7855 m
    public static final double kMaxErrorInElbow = 1_000;

    public static final double kOffsetFactor = 52.4 / 1; // 217.35 / 2

    public static final int kCloseEnoughTicks = 2000;

    // Elbow Positions
    public static final double kJogElbowTicks = 500.0;
    public static final double kIntakeStageElbow = -14_465;
    public static final double kIntakeElbow = -21_767; // -21_087
    public static final double kStowElbow = 0;
    public static final double kFloorElbow = 21_289; // 43_214
    public static final double kLevelOneElbow = 21_548;
    public static final double kLevelTwoConeElbow = 60_097; // 60_097 // FIXME
    public static final double kLevelTwoCubeElbow = 60_097;
    public static final double kLevelThreeConeElbow = 84_553; // 84_553 // FIXME
    public static final double kLevelThreeCubeElbow = 84_106;
    public static final double kShelfElbow = 41_000;
    public static final double kFloorElbowSweep = 16_876;
    public static final double kAutoLevelThreeCubeElbow = 65_497; // 64_297
    public static final double kAutoLevelTwoConeElbow = 60_097;
    public static final double kAutoLevelTwoCubeElbow = 36_500;
    public static final double kAutoLevelThreeConeElbow = 84_553;

    public static final double kRetrieveGamepiecePercentOutput = 0.2;

    // Parallel Movement of Elbow Positions
    public static final double kAboveConeNodeParallelAllowed = 124_638; // NOT USED
    public static final double kFloorPickupParallelAllowed = 5_000; // 30_000
    public static final double kStowToShelfParallelAllowed = 20_000; // FIXME
    public static final double kStowToLowParallelAllowed = kLevelOneElbow;
    public static final double kStowToMidParallelAllowed = 27_000;
    public static final double kStowToHighParallelAllowed = kStowToMidParallelAllowed;
    public static final double kStowToIntakeStageParallelAllowed = -10_000;
    public static final double kIntakeStageToIntakeParallelAllowed = kIntakeElbow;

    public static final double kElbowTeleMotionCruiseVelocity = 12_000.0; // 13_000
    public static final double kElbowTeleMotionAcceleration = 35_000.0; // 38_000
    public static final double kElbowAutoMotionCruiseVelocity = 13_000.0;
    public static final double kElbowAutoMotionAcceleration = 50_000.0;

    public static TalonFXConfiguration getElbowFalonConfig() {

      TalonFXConfiguration elbowConfig = new TalonFXConfiguration();

      elbowConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 10, 60, 1.0);
      elbowConfig.voltageMeasurementFilter = 32;
      elbowConfig.voltageCompSaturation = 12;
      elbowConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      elbowConfig.velocityMeasurementWindow = 64;
      elbowConfig.neutralDeadband = 0.01;

      elbowConfig.slot0.kP = 0.3;
      elbowConfig.slot0.kI = 0.002;
      elbowConfig.slot0.kD = 20.0;
      elbowConfig.slot0.kF = 0.053;
      elbowConfig.slot0.integralZone = 400.0;
      elbowConfig.slot0.maxIntegralAccumulator = 40_000.0;
      elbowConfig.slot0.allowableClosedloopError = 150.0;

      elbowConfig.motionAcceleration = kElbowAutoMotionAcceleration; // 50_000
      elbowConfig.motionCruiseVelocity = kElbowAutoMotionCruiseVelocity;
      elbowConfig.forwardSoftLimitEnable = true;
      elbowConfig.forwardSoftLimitThreshold = kForwardSoftLimit;
      elbowConfig.reverseSoftLimitEnable = true;
      elbowConfig.reverseSoftLimitThreshold = kReverseSoftLimit;

      return elbowConfig;
    }
  }

  public static final double kCanConfigTimeout = 10; // ms

  public static final class ShoulderConstants {
    // FIXME old: 1836

    public static final int kShoulderId = 30; // FIXME
    public static final int kFollowerShoulderId = 34; // FIXME

    public static final double kMaxFwd = 8_000; // 5000
    public static final double kMaxRev = -3_000; // -100
    public static final double kMaxTwistTicks = 750.0;
    public static final double kTwistBy = 25.0;
    public static final double kTimeTwist = 5.0;

    public static final double kZeroDegs = 0; // FIXME

    public static final double kTicksPerDeg = 213.3; // old: 142.2 (ratio = 1.5)

    public static final double kAllowedError = 450; // old: 300

    public static final double kShoulderLen = 0.20; // a 0.21
    public static final double kShoulderLowerToElevatorLowerPivotDist = 0.242; // d 0.245
    public static final double kShoulderUpperToElevatorUpperPivotDist = 0.1975; // b
    public static final double kElevatorPivotDist = 0.342; // c (old 0.27)
    public static final double kElevatorBaseToPivot = 0.05; // f (old 0.065)
    public static final double kElevatorBaseToElevatorUpperPivot = 0.35; // g (old 0.23

    public static final double kOffsetDegs =
        Math.toDegrees(Math.asin(0.06 / kShoulderLowerToElevatorLowerPivotDist));

    // Shoulder Positions
    public static final double kIntakeShoulder = 0; // 0
    public static final double kStowShoulder = 0; // 0
    public static final double kFloorShoulder = 5_250; // 3500
    public static final double kLevelOneShoulder = 0; // 3000
    public static final double kLevelTwoConeShoulder = 1_285; // 1_085 // 723 // FIXME
    public static final double kLevelTwoCubeShoulder = 1_085; // 723
    public static final double kLevelThreeConeShoulder = 6_150; // 5_850 // 3900 // FIXME
    public static final double kLevelThreeCubeShoulder = kLevelThreeConeShoulder;
    public static final double kShelfShoulder = -2_800; // 0
    public static final double kAutoLevelTwoConeShoulder = 1_085; // 723
    public static final double kAutoLevelTwoCubeShoulder = 0.0;
    public static final double kAutoLevelThreeCubeShoulder = 1_885; // 1_485
    public static final double kAutoLevelThreeConeShoulder = 5_850; // 3900

    // Allow Parallel Movement Ticks
    public static final double kFloorPickupParallelAllowed = 2_000;
    public static final double kStowToLowParallelAllowed = kLevelOneShoulder;
    public static final double kFloorToStowParallelAllowed = kStowShoulder;
    public static final double kScoreToStowParallelAllowed = kStowShoulder;
    public static final double kHighToStowParallelAllowed = 3_000;
    public static final double kIntakeToStowParallelAllowed = kStowShoulder;
    public static final double kShelfToStowParallelAllowed = kStowShoulder;

    public static final double kShoulderTeleMotionCruiseVelocity = 1_200.0;
    public static final double kShoulderTeleMotionAcceleration = 2_000.0;
    public static final double kShoulderAutoMotionCruiseVelocity = 1_200.0;
    public static final double kShoulderAutoMotionAcceleration = 5_000.0;

    public static TalonSRXConfiguration getShoulderTalonConfig() {
      TalonSRXConfiguration shoulderConfig = new TalonSRXConfiguration();

      shoulderConfig.slot0.kP = 2.0; // OLD 5.0
      shoulderConfig.slot0.kI = 0.0;
      shoulderConfig.slot0.kD = 40.0; // OLD 10.0
      shoulderConfig.slot0.kF = 0.7; // OLD 2.5
      shoulderConfig.slot0.integralZone = 0;
      shoulderConfig.slot0.maxIntegralAccumulator = 0;
      shoulderConfig.slot0.allowableClosedloopError = 0;
      shoulderConfig.motionCruiseVelocity = kShoulderAutoMotionCruiseVelocity; // 375
      shoulderConfig.motionAcceleration = kShoulderAutoMotionAcceleration; // 1000

      shoulderConfig.forwardSoftLimitEnable = true;
      shoulderConfig.forwardSoftLimitThreshold = kMaxFwd;
      shoulderConfig.reverseSoftLimitEnable = true;
      shoulderConfig.reverseSoftLimitThreshold = kMaxRev;
      shoulderConfig.neutralDeadband = 0.04; // 0.01
      shoulderConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      shoulderConfig.velocityMeasurementWindow = 64;
      shoulderConfig.voltageCompSaturation = 12;
      shoulderConfig.voltageMeasurementFilter = 32;

      return shoulderConfig;
    }

    public static SupplyCurrentLimitConfiguration getShoulderTalonSupplyLimitConfig() {
      SupplyCurrentLimitConfiguration shoulderSupplyConfig = new SupplyCurrentLimitConfiguration();

      shoulderSupplyConfig.currentLimit = 8; // 10
      shoulderSupplyConfig.triggerThresholdCurrent = 30; // 10
      shoulderSupplyConfig.triggerThresholdTime = 0.5; // 0.04
      shoulderSupplyConfig.enable = true;

      return shoulderSupplyConfig;
    }
  }

  public static final class IntakeConstants {

    public static final int kIntakeFalconID = 20;
    public static final int kExtendTalonID = 21;

    public static final double kStartNextAxisIntakeStage = 500;
    public static final double kAllowedError = 150; // FIXME
    public static final int kCloseEnoughTicks = 150;
    public static final int kExtendPosTicks = -2_000; // -2_000
    public static final int kRetractPosTicks = 0;
    public static final int kPickupPosTicks = kExtendPosTicks; // -1_000

    public static final double kIntakeDelay = 0.0;
    public static final double kIntakeSpeed = 0.45; // -0.35
    public static final double kIntakeEjectSpeed = -0.3;
    public static final double kEjectTimerDelaySec = 3;
    public static final double kIntakePickupDelaySec = 0.1;
    public static final int kBeamBreakStableCounts = 2;

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
      talonConfig.motionAcceleration = 2_000;

      return talonConfig;
    }

    public static SupplyCurrentLimitConfiguration getTalonSupplyLimitConfig() {
      SupplyCurrentLimitConfiguration extendSupplyConfig = new SupplyCurrentLimitConfiguration();

      extendSupplyConfig.currentLimit = 5.0;
      extendSupplyConfig.triggerThresholdCurrent = 8.0;
      extendSupplyConfig.triggerThresholdTime = 0.5;
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
    public static int kRollerTalonId = 41;

    public static final double kRollerConeHoldSpeed = 0.2;
    public static final double kRollerCubeHoldSpeed = 0.15;
    public static final double kRollerPickUp = 0.7;
    public static final double kRollerOff = 0.0;
    public static final double kRollerDrop = -0.1;
    public static final double kRollerShoot = -1.0;

    public static final double kMaxFwd = 1250; // 1100
    public static final double kMaxRev = -500; // -1000

    public static final double kHasPieceMinTicks = 450;

    public static final double kHandVelocityThreshold = 5;
    public static final double kHandZeroSpeed = 0.1;
    public static final double kZeroTargetSpeedTicksPer100ms = 5;
    public static final int kZeroStableCounts = 1592;
    public static final int kHasConeStableCounts = 2;
    public static final int kHasCubeStableCounts = 2;
    public static final int kHandVelStable = 50;

    public static final double kHandHoldingPercent = 0.1; // FIXME
    public static final double kHoldingVelocityThreshold = 50; // FIXME
    public static final int kHoldingStableCounts = 5; // FIXME
    public static final int kHoldingTickThreshold = 200;

    public static final double kAllowedError = 250; // 150

    public static final double kHandOpenPosition = kMaxRev;
    public static final double kIntakeOpenPosition = -500; // 50
    public static final double kCubeGrabbingPosition = 300;
    public static final double kCubeShootingPosition = 200;
    public static final double kStowPosition = 1_000;
    public static final double kFloorOpenPosition = 300;
    public static final double kShelfOpenPosition = 0;
    public static final double kConeGrabbingPosition = 1200; // old: 1650
    public static final double kConeVelLimit = 50;

    public static final double kRetrieveGamepiecePosition = 800;
    public static final double kRetrieveGamepieceRollerSpeed = 0.8;

    public static TalonSRXConfiguration getHandTalonConfig() {
      TalonSRXConfiguration handConfig = new TalonSRXConfiguration();

      handConfig.slot0.kP = 2.0;
      handConfig.slot0.kI = 0.0;
      handConfig.slot0.kD = 50.0;
      handConfig.slot0.kF = 0.85;
      handConfig.slot0.integralZone = 0;
      handConfig.slot0.maxIntegralAccumulator = 0;
      handConfig.slot0.allowableClosedloopError = 40;
      handConfig.motionCruiseVelocity = 1000;
      handConfig.motionAcceleration = 10_000;

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

    public static TalonSRXConfiguration getRolleConfig() {
      TalonSRXConfiguration rollerConfig = new TalonSRXConfiguration();
      rollerConfig.voltageCompSaturation = 12;
      rollerConfig.voltageMeasurementFilter = 32;
      rollerConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      rollerConfig.velocityMeasurementWindow = 64;
      rollerConfig.neutralDeadband = 0.01;
      return rollerConfig;
    }

    public static SupplyCurrentLimitConfiguration getHandSupplyLimitConfig() {
      return new SupplyCurrentLimitConfiguration(true, 2, 2, 0.3);
    }

    public static SupplyCurrentLimitConfiguration getHandZeroSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 5, 5, 0.1);
    }

    public static SupplyCurrentLimitConfiguration getRollerSupplyLimitConfig() {
      return new SupplyCurrentLimitConfiguration(true, 2, 2, 0.1);
    }
  }

  public static class CompConstants {
    // Drive
    public static final double kWheelDiameterInches = 3.0 * (496 / 500.0);

    // Elbow
    public static final int kElbowZeroTicks = 694; // 730

    // Shoulder
    public static final double kShoulderMainZeroTicks = 2050;
    public static final double kShoulderFollowerZeroTicks = 2057;

    // Intake
    public static final int kIntakeZeroTicks = 2790; // 2440 ->2540
    public static final double kExtendPosTicks = -2_100;

    // Hand
    public static final double kHandZeroTicks = 1230; // 686 ->976
  }

  public static class ProtoConstants {
    // drive
    public static final double kWheelDiameterInches = 3.0 * (490 / 500.0);

    // Elbow
    public static final int kElbowZeroTicks = 1135; // 1105

    // Shoulder
    public static final double kShoulderMainZeroTicks = 1909; // old: 1472
    public static final double kShoulderFollowerZeroTicks = 2152; // old: 3167

    // Intake
    public static final int kIntakeZeroTicks = 1690;
    public static final double kExtendPosTicks = -1_950;

    // Hand
    public static final double kHandZeroTicks = 873; // 879
  }
}
