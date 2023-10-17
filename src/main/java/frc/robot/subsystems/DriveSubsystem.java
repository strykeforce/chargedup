package frc.robot.subsystems;

import static frc.robot.Constants.kTalonConfigTimeout;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.VisionSubsystem.VisionStates;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Set;
import net.consensys.cava.toml.Toml;
import net.consensys.cava.toml.TomlArray;
import net.consensys.cava.toml.TomlParseResult;
import net.consensys.cava.toml.TomlTable;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.Follow;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Timed;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private final SwerveDrive swerveDrive;
  private final HolonomicDriveController holonomicController;
  private final ProfiledPIDController omegaController;
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController omegaAutoDriveController;
  private final ProfiledPIDController xAutoDriveController;
  private final ProfiledPIDController yAutoDriveController;
  private double xCalcAutoPickup = 0;
  private double yCalcAutoPickup = 0;
  private double xCalcError = 0;
  private double yCalcError = 0;
  private double xCalc;
  private double yCalc;
  private double omegaCalc;
  private RobotStateSubsystem robotStateSubsystem;

  // HEALTHCHECK FIX
  @HealthCheck
  @Timed(
      percentOutput = {0.2, 1, -0.2, -1},
      duration = 2)
  private TalonSRX azimuthZero;

  @HealthCheck
  @Follow(leader = 0)
  private TalonSRX azimuthOne;

  @HealthCheck
  @Follow(leader = 0)
  private TalonSRX azimuthTwo;

  @HealthCheck
  @Follow(leader = 0)
  private TalonSRX azimuthThree;

  // HEALTHCHECK FIX
  @HealthCheck
  @Timed(
      percentOutput = {0.2, 1, -0.2, -1},
      duration = 2)
  private TalonFX driveZero;

  @HealthCheck
  @Follow(leader = 10)
  private TalonFX driveOne;

  @HealthCheck
  @Follow(leader = 10)
  private TalonFX driveTwo;

  @HealthCheck
  @Follow(leader = 10)
  private TalonFX driveThree;

  // Grapher Variables
  private ChassisSpeeds holoContOutput = new ChassisSpeeds();
  private State holoContInput = new State();
  private Rotation2d holoContAngle = new Rotation2d();
  private Double trajectoryActive = 0.0;
  private double[] lastVelocity = new double[3];
  private PoseEstimatorOdometryStrategy odometryStrategy;
  private TimestampedPose timestampedPose =
      new TimestampedPose(RobotController.getFPGATime(), new Pose2d());
  public VisionSubsystem visionSubsystem;
  private double distanceVisionWheelOdom = 0.0;
  public boolean autoDriving = false,
      visionUpdates = false; // FIXME NEEDS TO BE TRUE FOR EVERY OTHER BRANCH
  private Pose2d endAutoDrivePose;
  private double lastXSpeed = 0.0, lastYSpeed = 0.0, speedMPSglobal = 0.0;
  private Timer autoDriveTimer = new Timer();
  private double lastAutoDriveTimer = 0;
  private Rotation2d desiredHeading;
  private Trajectory place;
  public DriveStates currDriveState = DriveStates.IDLE;
  private boolean isShelf;
  private Constants constants;
  private AHRS ahrs;
  public boolean autoBalanceGyroActive = false;
  private double autoBalanceStableCounts = 0;
  private Timer autoBalanceTimer = new Timer();
  private Timer autoBalancePulseTimer = new Timer();
  private Timer autoBalanceRecoveryTimer = new Timer();
  public boolean autoBalanceReadjust = false;
  public boolean isOnAllianceSide;
  public boolean autoBalanceTempStable = false;
  public double tempRoll;
  private double autoBalanceAvgCount = 0;
  private double sumRoll = 0;
  private double avgStartingRoll = 0;
  private boolean yawAdjustmentActive = false;
  private double recoveryStartingPitch;
  private boolean recoveryStartingPitchSet;
  private double autoBalanceGyroDirection = 0;
  private double yawAdjust = 0;
  // private boolean isAutoDriveFinished = false;

  public DriveSubsystem(Constants constants) {
    this.constants = constants;
    var moduleBuilder =
        new TalonSwerveModule.Builder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(constants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

    TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuthTalon.configFactoryDefault(kTalonConfigTimeout);
      azimuthTalon.configAllSettings(DriveConstants.getAzimuthTalonConfig(), kTalonConfigTimeout);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      if (i == 0) azimuthZero = azimuthTalon;
      else if (i == 1) azimuthOne = azimuthTalon;
      else if (i == 2) azimuthTwo = azimuthTalon;
      else if (i == 3) azimuthThree = azimuthTalon;

      var driveTalon = new TalonFX(i + 10);
      driveTalon.configFactoryDefault(kTalonConfigTimeout);
      driveTalon.configAllSettings(
          DriveConstants.getDriveTalonConfig(), Constants.kTalonConfigTimeout);
      driveTalon.enableVoltageCompensation(true);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      if (i == 0) driveZero = driveTalon;
      else if (i == 1) driveOne = driveTalon;
      else if (i == 2) driveTwo = driveTalon;
      else if (i == 3) driveThree = driveTalon;

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .build();

      swerveModules[i].loadAndSetAzimuthZeroReference();
    }
    ahrs = new AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, (byte) 200);
    swerveDrive = new SwerveDrive(ahrs, swerveModules);
    swerveDrive.resetGyro();
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));

    // Setup Holonomic Controller
    omegaAutoDriveController =
        new ProfiledPIDController(
            DriveConstants.kPAutoDriveOmega,
            DriveConstants.kIAutoDriveOmega,
            DriveConstants.kDAutoDriveOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kAutoDriveMaxAccelOmega));
    omegaAutoDriveController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
    // xAutoDriveController =
    //     new PIDController(
    //         DriveConstants.kPAutoDrive, DriveConstants.kIAutoDrive, DriveConstants.kDAutoDrive);
    // yAutoDriveController =
    //     new PIDController(
    //         DriveConstants.kPAutoDrive, DriveConstants.kIAutoDrive, DriveConstants.kDAutoDrive);
    xAutoDriveController =
        new ProfiledPIDController(
            DriveConstants.kPAutoDrive,
            DriveConstants.kIAutoDrive,
            DriveConstants.kDAutoDrive,
            new TrapezoidProfile.Constraints(
                DriveConstants.kAutoDriveMaxVelocity, DriveConstants.kAutoDriveMaxAccel));
    // // xAutoDriveController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    yAutoDriveController =
        new ProfiledPIDController(
            DriveConstants.kPAutoDrive,
            DriveConstants.kIAutoDrive,
            DriveConstants.kDAutoDrive,
            new TrapezoidProfile.Constraints(
                DriveConstants.kAutoDriveMaxVelocity, DriveConstants.kAutoDriveMaxAccel));
    // yAutoDriveController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    omegaController =
        new ProfiledPIDController(
            DriveConstants.kPOmega,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmega));
    omegaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    xController =
        new PIDController(
            DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    // xController.setIntegratorRange(DriveConstants.kIMin, DriveConstants.kIMax);
    yController =
        new PIDController(
            DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    // yController.setIntegratorRange(DriveConstants.kIMin, DriveConstants.kIMax);
    holonomicController = new HolonomicDriveController(xController, yController, omegaController);
    // Disabling the holonomic controller makes the robot directly follow the trajectory output (no
    // closing the loop on x,y,theta errors)
    holonomicController.setEnabled(true);
    odometryStrategy =
        new PoseEstimatorOdometryStrategy(
            swerveDrive.getHeading(),
            new Pose2d(),
            swerveDrive.getKinematics(),
            Constants.VisionConstants.kStateStdDevs,
            Constants.VisionConstants.kLocalMeasurementStdDevs,
            Constants.VisionConstants.kVisionMeasurementStdDevs,
            getSwerveModulePositions());
    swerveDrive.setOdometry(odometryStrategy);
  }

  @BeforeHealthCheck
  public boolean followTalons() {
    azimuthOne.follow(azimuthZero);
    azimuthTwo.follow(azimuthZero);
    azimuthThree.follow(azimuthZero);
    driveOne.follow(driveZero);
    driveTwo.follow(driveZero);
    driveThree.follow(driveZero);
    return true;
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModule[] swerveModules = getSwerveModules();
    SwerveModulePosition[] temp = {null, null, null, null};
    for (int i = 0; i < 4; ++i) {
      temp[i] = swerveModules[i].getPosition();
    }
    return temp;
  }

  public double distancePose(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }

  public void setCalcValues(double a, double b, double c, double d) {
    xCalcAutoPickup = a;
    yCalcAutoPickup = b;
    xCalcError = c;
    yCalcError = d;
  }

  public void setRobotStateSubsystem(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  public double getVectorSpeed() {
    return Math.sqrt(
        Math.pow(getFieldRelSpeed().vxMetersPerSecond, 2)
            + Math.pow(getFieldRelSpeed().vyMetersPerSecond, 2));
  }

  public boolean canGetVisionUpdates() {
    return visionUpdates;
  }

  public void drivePickup(Pose2d desiredPose) {
    robotStateSubsystem.setLights(0.0, 1.0, 1.0);
    omegaAutoDriveController.reset(getPoseMeters().getRotation().getRadians());
    xAutoDriveController.reset(getPoseMeters().getX(), getFieldRelSpeed().vxMetersPerSecond);
    yAutoDriveController.reset(getPoseMeters().getY(), getFieldRelSpeed().vyMetersPerSecond);
    xAutoDriveController.setP(DriveConstants.kPAutoPickup);
    yAutoDriveController.setP(DriveConstants.kPAutoPickup);
    omegaAutoDriveController.setP(DriveConstants.kPAutoPickup);
    setAutoDriving(true);
    endAutoDrivePose =
        new Pose2d(
            new Translation2d(
                (robotStateSubsystem.isBlueAlliance()
                    ? desiredPose.getX()
                    : Constants.RobotStateConstants.kFieldMaxX - desiredPose.getX()),
                desiredPose.getY()),
            new Rotation2d());
    if (visionSubsystem.isCameraWorking())
      visionSubsystem.resetOdometry(endAutoDrivePose, robotStateSubsystem.isBlueAlliance());
    logger.info("Starting auto pickup going to {}", endAutoDrivePose);
    logger.info("{} -> AUTO_DRIVE", currDriveState);
    currDriveState = DriveStates.AUTO_DRIVE;
  }

  public void driveToPose(TargetCol targetCol) {
    omegaAutoDriveController.reset(getPoseMeters().getRotation().getRadians());
    xAutoDriveController.reset(getPoseMeters().getX(), getFieldRelSpeed().vxMetersPerSecond);
    yAutoDriveController.reset(getPoseMeters().getY(), getFieldRelSpeed().vyMetersPerSecond);
    xAutoDriveController.setP(DriveConstants.kPAutoDrive);
    yAutoDriveController.setP(DriveConstants.kPAutoDrive);
    omegaAutoDriveController.setP(DriveConstants.kPAutoDrive);
    if (Math.abs(getFieldRelSpeed().vxMetersPerSecond) // FIXME TEMP CHECK IF STATEMENT
                <= DriveConstants.kMaxSpeedToAutoDrive
            && Math.abs(getFieldRelSpeed().vyMetersPerSecond) <= DriveConstants.kMaxSpeedToAutoDrive
            && (visionSubsystem.getState() == VisionStates.trustWheels
                || robotStateSubsystem.getIsAuto())
            && ((true))
        || robotStateSubsystem.getIsAuto()) {
      setAutoDriving(true);
      if (robotStateSubsystem.getGamePiece() != GamePiece.NONE) {
        endAutoDrivePose =
            robotStateSubsystem.getAutoPlaceDriveTarget(getPoseMeters().getY(), targetCol);
        logger.info("AutoDrive Scoring Gamepiece.");
        // if (robotStateSubsystem.autoDriveYawRight(getPoseMeters().getY()) != 0)
        // yawAdjustmentActive = false; // adjust yaw to get a better Odometry reading
      } else {
        // endAutoDrivePose =
        //     robotStateSubsystem.getShelfPosAutoDrive(
        //         targetCol, robotStateSubsystem.isBlueAlliance());
        // logger.info("AutoDrive Going to Shelf.");
        endAutoDrivePose = getPoseMeters();
      }
      logger.info(
          "endAutoDrivePose X: {}, endAutoDrivePose Y: {}",
          endAutoDrivePose.getX(),
          endAutoDrivePose.getY());
      // endAutoDrivePose =
      //     robotStateSubsystem.getAutoPlaceDriveTarget(getPoseMeters().getY(), targetCol);
      autoDriveTimer.reset();
      autoDriveTimer.start();
      logger.info("{} -> AUTO_DRIVE", currDriveState);
      currDriveState = DriveStates.AUTO_DRIVE;
    } else {
      logger.info(
          "Autodrive Failed. failedSpeed: {}, failedVision: {}",
          Math.abs(getFieldRelSpeed().vxMetersPerSecond) // FIXME TEMP CHECK IF STATEMENT
                  >= DriveConstants.kMaxSpeedToAutoDrive
              && Math.abs(getFieldRelSpeed().vyMetersPerSecond)
                  >= DriveConstants.kMaxSpeedToAutoDrive,
          visionSubsystem.getState() == VisionStates.trustWheels);
      logger.info("{} -> AUTO_DRIVE_FAILED", currDriveState);
      currDriveState = DriveStates.AUTO_DRIVE_FAILED;
    }
  }

  public boolean isAutoDriveFinished() {
    // autoDriving &&
    // if (place != null) {
    // if (autoDriveTimer.hasElapsed(place.getTotalTimeSeconds())) setAutoDriving(false);
    return inScorePosition();
    // return autoDriveTimer.hasElapsed(place.getTotalTimeSeconds());
    // } else return false;
  }

  public double getFinalAutoDriveTime() {
    return lastAutoDriveTimer;
  }

  public boolean inScorePosition() {
    Transform2d pathError = endAutoDrivePose.minus(getPoseMeters());
    if (Math.abs(pathError.getX()) <= DriveConstants.kPathErrorThreshold
        && Math.abs(pathError.getY())
            <= DriveConstants
                .kPathErrorThreshold) { // && Math.abs(pathError.getRotation().getDegrees())
      // <= DriveConstants.kPathErrorOmegaThresholdDegrees) {
      return true;
    } else return false;
  }

  public void autoDrive(boolean isShelf, TargetCol targetCol) {
    this.isShelf = isShelf;
    if (robotStateSubsystem.isBlueAlliance()) desiredHeading = new Rotation2d(0.0);
    else desiredHeading = new Rotation2d(Math.PI);
    if (isShelf)
      desiredHeading = new Rotation2d(desiredHeading.getRadians() == Math.PI ? 0.0 : Math.PI);
    setEnableHolo(true);
    resetHolonomicController();
    logger.info("Moving to place");
    TrajectoryConfig config = new TrajectoryConfig(2.5, 1.5);
    config.setEndVelocity(0);
    config.setStartVelocity(0.0);
    ArrayList<Translation2d> points = new ArrayList<>();
    Pose2d endPose = new Pose2d();
    double startAngle = robotStateSubsystem.isBlueAlliance() ? Math.PI : 0.0;
    if (isShelf) startAngle = startAngle == 0.0 ? Math.PI : 0.0;
    Pose2d start =
        new Pose2d(
            new Translation2d(getPoseMeters().getX(), getPoseMeters().getY()),
            new Rotation2d(startAngle));
    if (!isShelf)
      endPose = robotStateSubsystem.getAutoPlaceDriveTarget(getPoseMeters().getY(), targetCol);
    else
      endPose =
          robotStateSubsystem.getShelfPosAutoDrive(targetCol, robotStateSubsystem.isBlueAlliance());
    logger.info("Autodriving to: {}, isShelf: {}", endPose, isShelf);
    if (!isShelf)
      points.add(
          new Translation2d(
              (getPoseMeters().getX() + endPose.getX()) / 2,
              (getPoseMeters().getY() + endPose.getY()) / 2));
    else points.add(new Translation2d((start.getX() * 0.2 + endPose.getX() * 0.8), endPose.getY()));
    visionUpdates = false;
    place = TrajectoryGenerator.generateTrajectory(start, points, endPose, config);
    autoDriveTimer.reset();
    autoDriveTimer.start();
    logger.info("{} -> AUTO_DRIVE", currDriveState);
    currDriveState = DriveStates.AUTO_DRIVE;
    grapherTrajectoryActive(true);
    // calculateController(place.sample(autoDriveTimer.get()), desiredHeading);
  }
  // private void autoDrive() {
  //   Pose2d currentPose = getPoseMeters();
  //   Rotation2d robotRotation = getGyroRotation2d();

  //   // holoContInput =
  //   double xCalc = xAutoDriveController.calculate(currentPose.getX(),endAutoDrivePose.getX());
  //   double yCalc = xAutoDriveController.calculate(currentPose.getY(),endAutoDrivePose.getY());
  //   double omegaCalc =
  //       omegaAutoDriveController.calculate(
  //           MathUtil.angleModulus(robotRotation.getRadians()),
  //           robotStateSubsystem.getAllianceColor() == Alliance.Blue ? 0.0 : Math.PI);

  //   move(xCalc, yCalc, omegaCalc, true);

  //   // logger.info("X accel {}, Y accel", moveTranslation.getX(), moveTranslation.getY());
  //   // logger.info(" X Dif: {}, Y Dif: {}", poseDifference.getX(), poseDifference.getY());

  // }

  public boolean isAutoDriving() {
    return autoDriving;
  }

  public void setDoYawAdjust(boolean doYawAdjust, double yawAdjustBy) {
    yawAdjustmentActive = doYawAdjust;
    yawAdjust = yawAdjustBy;
  }

  public void setDriveState(DriveStates driveStates) {
    logger.info("{} -> {}", currDriveState, driveStates);
    currDriveState = driveStates;
  }

  public DriveStates getDriveState() {
    return currDriveState;
  }

  public void setAutoDriving(boolean autoDrive) {
    autoDriving = autoDrive;
  }

  public void autoBalance(boolean isOnAllianceSide) {
    // On alliance side of charge station, drive Positive X
    autoBalanceAvgCount = 0;
    sumRoll = 0;
    avgStartingRoll = 0;
    this.isOnAllianceSide = isOnAllianceSide;
    tempRoll = getGyroPitch();
    logger.info(
        "Starting AutoBalance: tempRoll: {}, Alliance: {}, isOnAllianceSide: {}",
        tempRoll,
        robotStateSubsystem.getAllianceColor(),
        isOnAllianceSide);
    if (isOnAllianceSide) {
      move(DriveConstants.kAutoBalanceFinalDriveVel, 0, 0, false);
      logger.info("DRIVING POSITIVE LIKE ITS SUPPOSED TO");
    }
    // NOT On alliance side of charge station, drive Negative X
    else move(-DriveConstants.kAutoBalanceFinalDriveVel, 0, 0, false);
    logger.info("{} -> AUTO_BALANCE_EDGE", currDriveState);
    currDriveState = DriveStates.AUTO_BALANCE_EDGE;
  }

  public void pulseAutoBalance(boolean isOnAllianceSide) {
    this.isOnAllianceSide = isOnAllianceSide;
    autoBalancePulseTimer.start();
    autoBalancePulseTimer.reset();
    tempRoll = Math.abs(getGyroPitch());
    move(0.0, 0.0, 0.0, false);
    logger.info("{} -> AUTO_BALANCE_CHECK", currDriveState);
    currDriveState = DriveStates.AUTO_BALANCE_CHECK;
  }

  public boolean isWithinThresholdAutoBalance(double threshold) {
    return (ahrs.getRoll() <= threshold && ahrs.getRoll() >= -threshold);
  }

  @Override
  public void periodic() {
    // Update swerve module states every robot loop
    swerveDrive.periodic();
    switch (currDriveState) {
      case IDLE:
        break;
      case AUTO_DRIVE:
        if (isAutoDriveFinished()) {
          visionSubsystem.setOdomAutoBool(false);
          drive(0, 0, 0);
          visionUpdates = true;
          setAutoDriving(false);
          logger.info("End Trajectory {}", autoDriveTimer.get());
          lastAutoDriveTimer = autoDriveTimer.get();
          autoDriveTimer.stop();
          autoDriveTimer.reset();
          logger.info("{} -> AUTO_DRIVE_FINISHED", currDriveState);
          currDriveState = DriveStates.AUTO_DRIVE_FINISHED;
          break;
        }
        if (autoDriving
            && visionUpdates
            && autoDriveTimer.hasElapsed(DriveConstants.kArmToAutoDriveDelaySec)) {
          lastAutoDriveTimer = autoDriveTimer.get();

          double tempYaw = 0;
          if (robotStateSubsystem.getAllianceColor() == Alliance.Red) tempYaw = Math.PI;
          if (yawAdjustmentActive) tempYaw += Rotation2d.fromDegrees(yawAdjust).getRadians();

          logger.info(
              "YawAdjust: {}, target Yaw: {}, Current Yaw: {}",
              yawAdjustmentActive,
              Rotation2d.fromRadians(tempYaw).getDegrees(),
              getGyroRotation2d().getDegrees());
          double xCalc =
              xAutoDriveController.calculate(getPoseMeters().getX(), endAutoDrivePose.getX());
          double yCalc =
              yAutoDriveController.calculate(getPoseMeters().getY(), endAutoDrivePose.getY());
          double omegaCalc =
              omegaAutoDriveController.calculate(
                  MathUtil.angleModulus(getGyroRotation2d().getRadians()), tempYaw);
          // robotStateSubsystem.getAllianceColor() == Alliance.Blue ? 0.0 : Math.PI);
          // if (yawAdjustmentActive) {
          //   double tempYaw = 45;
          //   if (robotStateSubsystem.autoDriveYawRight(getPoseMeters().getY()) == 2)
          //     tempYaw = -tempYaw;
          //   else if (robotStateSubsystem.autoDriveYawRight(getPoseMeters().getY()) == 0) {
          //     logger.info(
          //         "Target Seen, Stop yawing, Yaw Angle: {}",
          //         getPoseMeters().getRotation().getDegrees());
          //     tempYaw = 0;
          //     yawAdjustmentActive = false;
          //   }
          //   tempYaw =
          //       robotStateSubsystem.getAllianceColor() == Alliance.Blue
          //           ? 0.0 + Math.toRadians(tempYaw)
          //           : Math.PI + Math.toRadians(tempYaw);
          //   if (Math.abs(Math.toDegrees(tempYaw) - getGyroRotation2d().getDegrees())
          //       <= DriveConstants.kAutoDriveAutoYawCloseEnough) {
          //     logger.info(
          //         "At Angle, Stop yawing, Yaw Angle: {}",
          //         getPoseMeters().getRotation().getDegrees());
          //     yawAdjustmentActive = false;
          //     tempYaw = 0;
          //   }
          //   omegaCalc =
          //       omegaAutoDriveController.calculate(
          //           MathUtil.angleModulus(getGyroRotation2d().getRadians()), tempYaw);
          //   logger.info("tempYaw: {}", Math.toDegrees(tempYaw));
          // }
          // logger.info("AutoPlace: xCalc: {}, yCalc: {}, omegaCalc: {}", xCalc, yCalc, omegaCalc);
          this.xCalc = xCalc;
          this.yCalc = yCalc;
          this.omegaCalc = omegaCalc;
          logger.info(
              "Moving X : {} | Moving Y : {} | \nCurrent Pose {}", xCalc, yCalc, getPoseMeters());
          logger.info(
              "X controller: err: {}, goal: {}\n",
              xAutoDriveController.getPositionError(),
              xAutoDriveController.getGoal().position);
          move(xCalc, yCalc, omegaCalc, true);
          // calculateController(place.sample(autoDriveTimer.get()), desiredHeading);
        }
        if (autoDriving
            && !visionUpdates
            && (!visionSubsystem.getOdomAutoBool() || !visionSubsystem.isCameraWorking())) {
          visionSubsystem.setOdomAutoBool(false);
          drive(0, 0, 0);
          visionUpdates = true;
          setAutoDriving(false);
          logger.info("End Trajectory {}", autoDriveTimer.get());
          autoDriveTimer.stop();
          autoDriveTimer.reset();
          logger.info("{} -> AUTO_DRIVE_FAILED", currDriveState);
          setDriveState(DriveStates.AUTO_DRIVE_FAILED);
          break;
        }
        break;
      case AUTO_DRIVE_FAILED:
        // logger.info("Failed AutoDrive, Attempting to Move Yaw to 0");
        // double omegaCalc =
        //     omegaAutoDriveController.calculate(
        //         MathUtil.angleModulus(getGyroRotation2d().getRadians()),
        //         robotStateSubsystem.getAllianceColor() == Alliance.Blue ? 0.0 : Math.PI);
        // move(0, 0, omegaCalc, true);
        break;
      case AUTO_DRIVE_FINISHED:
        autoDriving = false;
        visionUpdates = true;
        // logger.info("Autodrive Finished");
        lastXSpeed = 0.0;
        speedMPSglobal = 0.0;
        lastYSpeed = 0.0;
        break;
      case AUTO_BALANCE_EDGE:
        if (Math.abs(getGyroPitch()) - Math.abs(tempRoll)
            >= DriveConstants.kAutoBalanceEdgeTriggerThreshold) {
          autoBalanceTimer.reset();
          autoBalanceTimer.start();
          logger.info("{} -> AUTO_BALANCE_DRIVE", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_DRIVE;
        }
        break;
      case AUTO_BALANCE_DRIVE:
        if (autoBalanceTimer.hasElapsed(DriveConstants.kAutoBalanceSlowdownTimeSec)) {
          logger.info("{} -> AUTO_BALANCE_AVERAGE", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_AVERAGE;
          autoBalanceTimer.restart();
          autoBalanceGyroDirection = getGyroPitch() - tempRoll;
        }
        break;
      case AUTO_BALANCE_AVERAGE:
        sumRoll += getGyroPitch();
        autoBalanceAvgCount++;
        logger.info("AutoBalanceAvgCount: {}", autoBalanceAvgCount);
        if (autoBalanceTimer.hasElapsed(DriveConstants.kAutoBalanceLoopFixTimer)) {
          avgStartingRoll = sumRoll / autoBalanceAvgCount;
          logger.info("Average Starting Roll: {}", avgStartingRoll);
          logger.info("{} -> AUTO_BALANCE", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE;
          if (isOnAllianceSide) move(DriveConstants.kAutoBalanceSlowDriveVel, 0, 0, false);
          else move(-DriveConstants.kAutoBalanceSlowDriveVel, 0, 0, false);
          autoBalanceTimer.reset();
        }
        break;
      case AUTO_BALANCE:
        if (Math.abs(avgStartingRoll) - Math.abs(getGyroPitch())
            >= DriveConstants.kAutoBalanceStopThresholdDegrees) {
          drive(0, 0, 0);
          xLock();
          logger.info(
              "AutoBalance Stop: Gyro Roll: {}, trigger Difference: {}",
              getGyroPitch(),
              Math.abs(avgStartingRoll) - Math.abs(getGyroPitch()));
          logger.info("{} -> AUTO_BALANCE_RECOVERY", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_RECOVERY;

          autoBalanceRecoveryTimer.reset();
          autoBalanceRecoveryTimer.start();
          recoveryStartingPitchSet = false;
        }
        break;
      case AUTO_BALANCE_RECOVERY:
        if (autoBalanceRecoveryTimer.hasElapsed(DriveConstants.kSettleTime)) {
          autoBalanceRecoveryTimer.stop();
          // isOnAllianceSide is true move positive
          if (!recoveryStartingPitchSet) {
            recoveryStartingPitch = getGyroPitch();
            recoveryStartingPitchSet = true;
          }
          // if (isOnAllianceSide)
          if ((Math.abs(tempRoll - getGyroPitch()) > 2)
              && (Math.abs(recoveryStartingPitch) - Math.abs(getGyroPitch()) < 1.5)) {
            // move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
            // if (((autoBalanceGyroDirection > 0) && (((getGyroPitch() - tempRoll) > 0) &&
            // isOnAllianceSide)) || ((autoBalanceGyroDirection < 0) && (((getGyroPitch() -
            // tempRoll) < 0) && isOnAllianceSide))) {}
            if ((autoBalanceGyroDirection > 0)) {
              // Positive Gyro Direction
              if ((getGyroPitch() - tempRoll) > 0) {
                if (isOnAllianceSide)
                  move(DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
                else move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
              } else if ((getGyroPitch() - tempRoll) < 0) {
                if (isOnAllianceSide)
                  move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
                else move(DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
              }
            } else if ((autoBalanceGyroDirection < 0)) {
              // negative gyro direction
              if ((getGyroPitch() - tempRoll) > 0) {
                if (isOnAllianceSide)
                  move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
                else move(DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
              } else if ((getGyroPitch() - tempRoll) < 0) {
                if (isOnAllianceSide)
                  move(DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
                else move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
              }
            } // 9 if pos or like -8 if neg
          } else {
            drive(0, 0, 0);
            xLock();

            logger.info(
                "AutoBalance Stop: Gyro Roll: {}, trigger Difference: {}",
                getGyroPitch(),
                Math.abs(avgStartingRoll) - Math.abs(getGyroPitch()));
            logger.info("{} -> AUTO_BALANCE_RECOVERY", currDriveState);
            currDriveState = DriveStates.AUTO_BALANCE_RECOVERY;

            autoBalanceRecoveryTimer.reset();
            autoBalanceRecoveryTimer.start();
            recoveryStartingPitchSet = false;
            // logger.info("{} -> AUTO_BALANCE_FINISHED", currDriveState);
            // currDriveState = DriveStates.AUTO_BALANCE_FINISHED;
          }
        }
        break;
      case AUTO_BALANCE_FINISHED:
        autoBalanceAvgCount = 0;
        sumRoll = 0;
        avgStartingRoll = 0;
        break;
      case AUTO_BALANCE_PULSE:
        if (!autoBalanceTempStable
            && autoBalancePulseTimer.hasElapsed(DriveConstants.kPulseAutoBalanceTime)) {
          if ((!isOnAllianceSide && robotStateSubsystem.getAllianceColor() == Alliance.Blue)
              || (robotStateSubsystem.getAllianceColor() == Alliance.Red && isOnAllianceSide))
            move(DriveConstants.kHoldSpeed, 0, 0, false);
          else move(-DriveConstants.kHoldSpeed, 0, 0, false);
          logger.info("{} -> AUTO_BALANCE_CHECK", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_CHECK;
          autoBalancePulseTimer.reset();
        }
        break;
      case AUTO_BALANCE_CHECK:
        if (isWithinThresholdAutoBalance(DriveConstants.kAutoBalanceCloseEnoughDeg)) {
          autoBalanceStableCounts++;
          autoBalanceTempStable = true;
        } else {
          autoBalanceStableCounts = 0;
        }
        if (autoBalancePulseTimer.hasElapsed(DriveConstants.kPauseAutoBalanceTime)
            && !(autoBalanceStableCounts >= DriveConstants.kAutoBalanceStableCount)) {
          double tempSpeed = DriveConstants.kPulseSpeed;
          if (Math.abs(getGyroPitch()) - 2.9 <= 10) {
            logger.info("AutoBalance Slowing to temp speed");
            tempSpeed = 0.3;
          }
          logger.info("{} -> AUTO_BALANCE_PULSE", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_PULSE;
          autoBalancePulseTimer.reset();
          if ((!isOnAllianceSide && robotStateSubsystem.getAllianceColor() == Alliance.Blue)
              || (robotStateSubsystem.getAllianceColor() == Alliance.Red && isOnAllianceSide))
            move(tempSpeed, 0, 0, false);
          else move(-tempSpeed, 0, 0, false);
          break;
        }
        if ((autoBalanceStableCounts >= DriveConstants.kAutoBalanceStableCount)) {
          logger.info("{} -> AUTO_BALANCE_XLOCK", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_XLOCK;
          drive(0, 0, 0);
          xLock();
        }
        break;
      case AUTO_BALANCE_XLOCK:
        break;
      default:
        break;
    }
  }

  public double getSpeedMPS() {
    return speedMPSglobal;
  }

  public double getLastXSpeed() {
    return lastXSpeed;
  }

  public double getLastYSpeed() {
    return lastYSpeed;
  }

  public double distanceOdometryVision(Pose2d vision) {
    return distancePose(vision, swerveDrive.getPoseMeters());
  }

  public void setVisionSubsystem(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
  }

  public void setOdometry(Rotation2d Odom) {
    swerveDrive.setOdometry(null);
  }
  // Open-Loop Swerve Movements
  public void drive(double vXmps, double vYmps, double vOmegaRadps) {
    swerveDrive.drive(vXmps, vYmps, vOmegaRadps, true);
  }
  // Closed-Loop (Velocity Controlled) Swerve Movement
  public void move(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    swerveDrive.move(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
  }

  public void setGyroOffset(Rotation2d rotation) {
    swerveDrive.setGyroOffset(rotation);
  }

  public void teleResetGyro() {
    logger.info("Driver Joystick: Reset Gyro");
    double gyroResetDegs = robotStateSubsystem.getAllianceColor() == Alliance.Blue ? 0.0 : 180.0;
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(gyroResetDegs));
    swerveDrive.resetGyro();
    swerveDrive.resetOdometry(
        new Pose2d(
            swerveDrive.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(gyroResetDegs)));
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    logger.info("reset odometry with: {}", pose);
  }

  public void resetOdometryNoLog(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Rotation2d getGyroRotation2d() {
    return swerveDrive.getHeading();
  }

  public double getGyroPitch() {
    return ahrs.getPitch();
  }

  public boolean isNavxWorking() {
    return ahrs.isConnected();
  }

  public void lockZero() {
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(0.0));
    }
  }

  public void xLock() {
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      if (i == 0 || i == 3) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(45.0));
      }
      if (i == 1 || i == 2) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(-45.0));
      }
    }
  }

  public SwerveModule[] getSwerveModules() {
    return swerveDrive.getSwerveModules();
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    TalonSwerveModule[] swerveModules = (TalonSwerveModule[]) swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }

    return swerveModuleStates;
  }

  public void resetHolonomicController() {
    xController.reset();
    yController.reset();
    omegaController.reset(getGyroRotation2d().getRadians());
  }

  public ChassisSpeeds getFieldRelSpeed() {
    SwerveDriveKinematics kinematics = swerveDrive.getKinematics();
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; ++i) {
      swerveModuleStates[i] = swerveModules[i].getState();
    }
    ChassisSpeeds roboRelSpeed = kinematics.toChassisSpeeds(swerveModuleStates);
    return new ChassisSpeeds(
        roboRelSpeed.vxMetersPerSecond * swerveDrive.getHeading().unaryMinus().getCos()
            + roboRelSpeed.vyMetersPerSecond * swerveDrive.getHeading().unaryMinus().getSin(),
        -roboRelSpeed.vxMetersPerSecond * swerveDrive.getHeading().unaryMinus().getSin()
            + roboRelSpeed.vyMetersPerSecond * swerveDrive.getHeading().unaryMinus().getCos(),
        roboRelSpeed.omegaRadiansPerSecond);
  }

  // Trajectory TOML Parsing
  public PathData generateTrajectory(String trajectoryName) {
    try {
      if (shouldFlip()) {
        logger.info("Flipping path");
      }
      TomlParseResult parseResult =
          Toml.parse(Paths.get("/home/lvuser/deploy/paths/" + trajectoryName + ".toml"));
      logger.info("Generating Trajectory: {}", trajectoryName);
      Pose2d startPose = parsePose2d(parseResult, "start_pose");
      startPose = apply(startPose);
      Pose2d endPose = parsePose2d(parseResult, "end_pose");
      endPose = apply(endPose);
      TomlArray internalPointsToml = parseResult.getArray("internal_points");
      ArrayList<Translation2d> path = new ArrayList<>();
      logger.info("Toml Internal Points Array Size: {}", internalPointsToml.size());

      // Create a table for each internal point and put them into a translation2d waypoint
      for (int i = 0; i < internalPointsToml.size(); i++) {
        TomlTable waypointToml = internalPointsToml.getTable(i);
        Translation2d waypoint =
            new Translation2d(waypointToml.getDouble("x"), waypointToml.getDouble("y"));
        waypoint = apply(waypoint);
        path.add(waypoint);
      }

      // Trajectory Config parsed from toml - any additional constraints would be added here

      TrajectoryConfig trajectoryConfig =
          new TrajectoryConfig(
              parseResult.getDouble("max_velocity"), parseResult.getDouble("max_acceleration"));
      logger.info("max velocity/acceleration worked");
      trajectoryConfig.setReversed(parseResult.getBoolean("is_reversed"));
      trajectoryConfig.setStartVelocity(parseResult.getDouble("start_velocity"));
      logger.info("start velocity worked");
      trajectoryConfig.setEndVelocity(parseResult.getDouble("end_velocity"));
      logger.info("end velocity worked");

      // Yaw degrees is seperate from the trajectoryConfig
      double yawDegrees = parseResult.getDouble("target_yaw");
      logger.info("target yaw worked");
      Rotation2d target_yaw = Rotation2d.fromDegrees(yawDegrees);
      target_yaw = apply(target_yaw);
      logger.info("Yaw is {}", target_yaw);

      // Create a the generated trajectory and return it along with the target yaw
      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);

      logger.info("Total Time: {}", trajectoryGenerated.getTotalTimeSeconds());
      return new PathData(target_yaw, trajectoryGenerated);
    } catch (Exception error) {
      logger.error(error.toString());
      logger.error("Path {} not found - Running Default Path", trajectoryName);

      // In the case of an error, set the trajectory to default
      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(
              DriveConstants.startPose2d,
              DriveConstants.getDefaultInternalWaypoints(),
              DriveConstants.endPose2d,
              DriveConstants.getDefaultTrajectoryConfig());

      return new PathData(getGyroRotation2d(), trajectoryGenerated);
    }
  }

  public Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(
          Constants.FieldConstants.kFieldLength - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  public Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          Constants.FieldConstants.kFieldLength - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  public Rotation2d apply(Rotation2d rotation) {
    logger.info(
        "initial target yaw: {}, cos: {}, sin: {}", rotation, rotation.getCos(), rotation.getSin());
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else return rotation;
  }

  private boolean shouldFlip() {
    return robotStateSubsystem.getAllianceColor() == Alliance.Red;
  }

  private Pose2d parsePose2d(TomlParseResult parseResult, String pose) {
    return new Pose2d(
        parseResult.getTable(pose).getDouble("x"),
        parseResult.getTable(pose).getDouble("y"),
        Rotation2d.fromDegrees(parseResult.getTable(pose).getDouble("angle")));
  }

  public Pose2d getPoseMeters() {
    return swerveDrive.getPoseMeters();
  }

  public void updateOdometryWithVision(Pose2d calculatedPose) {
    odometryStrategy.addVisionMeasurement(calculatedPose, Timer.getFPGATimestamp());
  }

  public void updateOdometryWithVision(Pose2d calculatedPose, double timestamp) {
    if (visionUpdates) odometryStrategy.addVisionMeasurement(calculatedPose, timestamp);
  }

  // Holonomic Controller
  public void calculateController(State desiredState, Rotation2d desiredAngle) {
    holoContInput = desiredState;
    holoContAngle = desiredAngle;
    holoContOutput = holonomicController.calculate(getPoseMeters(), desiredState, desiredAngle);
    // logger.info("input: {}, output: {}, angle: {}", holoContInput, holoContOutput, desiredAngle);
    move(
        holoContOutput.vxMetersPerSecond,
        holoContOutput.vyMetersPerSecond,
        holoContOutput.omegaRadiansPerSecond,
        false);
  }

  public void setEnableHolo(boolean enabled) {
    holonomicController.setEnabled(enabled);
    logger.info("Holonomic Controller Enabled: {}", enabled);
  }

  public double getGyroRate() {
    return swerveDrive.getGyroRate();
  }

  // Make whether a trajectory is currently active obvious on grapher
  public void grapherTrajectoryActive(Boolean active) {
    if (active) trajectoryActive = 1.0;
    else trajectoryActive = 0.0;
  }

  public enum DriveStates {
    IDLE,
    AUTO_DRIVE,
    AUTO_DRIVE_FINISHED,
    AUTO_DRIVE_FAILED,
    AUTO_BALANCE_EDGE,
    AUTO_BALANCE_DRIVE,
    AUTO_BALANCE_AVERAGE,
    AUTO_BALANCE,
    AUTO_BALANCE_RECOVERY,
    AUTO_BALANCE_FINISHED,
    AUTO_BALANCE_PULSE,
    AUTO_BALANCE_CHECK,
    AUTO_BALANCE_XLOCK;
  }

  @Override
  public void registerWith(@NotNull TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    swerveDrive.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Gyro Rotation2D(deg)", () -> swerveDrive.getHeading().getDegrees()),
        new Measure("Odometry X", () -> swerveDrive.getPoseMeters().getX()),
        new Measure("Odometry Y", () -> swerveDrive.getPoseMeters().getY()),
        new Measure(
            "Odometry Rotation2D(deg)",
            () -> swerveDrive.getPoseMeters().getRotation().getDegrees()),
        new Measure("Trajectory Vel", () -> holoContInput.velocityMetersPerSecond),
        new Measure("Trajectory Accel", () -> holoContInput.accelerationMetersPerSecondSq),
        new Measure("Trajectory X", () -> holoContInput.poseMeters.getX()),
        new Measure("Trajectory Y", () -> holoContInput.poseMeters.getY()),
        new Measure(
            "Trajectory Rotation2D(deg)",
            () -> holoContInput.poseMeters.getRotation().getDegrees()),
        new Measure("Desired Gyro Heading(deg)", () -> holoContAngle.getDegrees()),
        new Measure("Holonomic Cont Vx", () -> holoContOutput.vxMetersPerSecond),
        new Measure("Holonomic Cont Vy", () -> holoContOutput.vyMetersPerSecond),
        new Measure("Holonomic Cont Vomega", () -> holoContOutput.omegaRadiansPerSecond),
        new Measure("Trajectory Active", () -> trajectoryActive),
        new Measure("Timestamp X", () -> timestampedPose.getPose().getX()),
        new Measure("Timestamp Y", () -> timestampedPose.getPose().getY()),
        new Measure("Wheel 0 Angle", () -> getSwerveModuleStates()[0].angle.getDegrees()),
        new Measure("Wheel 0 Speed", () -> getSwerveModuleStates()[0].speedMetersPerSecond),
        new Measure("Wheel 1 Angle", () -> getSwerveModuleStates()[1].angle.getDegrees()),
        new Measure("Wheel 1 Speed", () -> getSwerveModuleStates()[1].speedMetersPerSecond),
        new Measure("Wheel 2 Angle", () -> getSwerveModuleStates()[2].angle.getDegrees()),
        new Measure("Wheel 2 Speed", () -> getSwerveModuleStates()[2].speedMetersPerSecond),
        new Measure("Wheel 3 Angle", () -> getSwerveModuleStates()[3].angle.getDegrees()),
        new Measure("Wheel 3 Speed", () -> getSwerveModuleStates()[3].speedMetersPerSecond),
        new Measure("FWD Vel", () -> lastVelocity[0]),
        new Measure("STR Vel", () -> lastVelocity[1]),
        new Measure("YAW Vel", () -> lastVelocity[2]),
        new Measure("Field Rel Velocity X", () -> getFieldRelSpeed().vxMetersPerSecond),
        new Measure("Field Rel Velocity Y", () -> getFieldRelSpeed().vyMetersPerSecond),
        new Measure("Gyro Rate", () -> getGyroRate()),
        new Measure("Auto Drive Speed X", () -> getLastXSpeed()),
        new Measure("Auto Drive Speed Y", () -> getLastYSpeed()),
        new Measure("Auto Drive End X", () -> endAutoDrivePose.getX()),
        new Measure("Auto Drive End Y", () -> endAutoDrivePose.getY()),
        new Measure("Auto Drive Timer", () -> getFinalAutoDriveTime()),
        new Measure("AutoDrive Goal X", () -> xAutoDriveController.getGoal().position),
        new Measure("AutoDrive Goal Y", () -> yAutoDriveController.getGoal().position),
        new Measure("AutoDrive Pos Error Y", () -> yAutoDriveController.getPositionError()),
        new Measure("AutoDrive Pos Error X", () -> xAutoDriveController.getPositionError()),
        new Measure("AutoDrive SetPoint Y", () -> yAutoDriveController.getSetpoint().position),
        new Measure("AutoDrive SetPoint X", () -> xAutoDriveController.getSetpoint().position),
        new Measure("AutoDrive Velocity Err Y", () -> yAutoDriveController.getVelocityError()),
        new Measure("AutoDrive Velocity Err X", () -> xAutoDriveController.getVelocityError()),
        new Measure("AutoDrive xCalc", () -> xCalc),
        new Measure("AutoDrive yCalc", () -> yCalc),
        new Measure("AutoDrive omegaCalc", () -> omegaCalc),
        new Measure("Auto Drive Timer", () -> autoDriveTimer.get()),
        new Measure("Robot Roll Deg", () -> getGyroPitch()),
        new Measure("Drive State", () -> currDriveState.ordinal()),
        new Measure("SpeedMPS AUTODRIVE", () -> getSpeedMPS()),
        new Measure("Magnetic Disturbance (GYRO)", () -> ahrs.isMagneticDisturbance() ? 1.0 : 0.0),
        new Measure("XCalc AutoPickUp", () -> xCalcAutoPickup),
        new Measure("YCalc AutoPickUo", () -> yCalcAutoPickup),
        new Measure("Auto Pickup X Error", () -> xCalcError),
        new Measure("Auto Pickup Y Error", () -> yCalcError));
  }
}
