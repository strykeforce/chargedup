package frc.robot.subsystems;

import static frc.robot.Constants.kTalonConfigTimeout;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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
  private RobotStateSubsystem robotStateSubsystem;

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

  public DriveSubsystem() {
    var moduleBuilder =
        new TalonSwerveModule.Builder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
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

      var driveTalon = new TalonFX(i + 10);
      driveTalon.configFactoryDefault(kTalonConfigTimeout);
      driveTalon.configAllSettings(
          DriveConstants.getDriveTalonConfig(), Constants.kTalonConfigTimeout);
      driveTalon.enableVoltageCompensation(true);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .build();

      swerveModules[i].loadAndSetAzimuthZeroReference();
    }

    swerveDrive = new SwerveDrive(swerveModules);
    swerveDrive.resetGyro();
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));

    // Setup Holonomic Controller
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
    yController =
        new PIDController(
            DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
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

  public void setRobotStateSubsystem(RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void periodic() {
    // Update swerve module states every robot loop
    swerveDrive.periodic();
    // TimestampedPose pose = visionSubsystem.odomNewPoseViaVision();
    // if (!DriverStation.isAutonomous()) {
    //   distanceVisionWheelOdom = distancePose(pose.getPose(), swerveDrive.getPoseMeters());
    //   timestampedPose.setPose(pose.getPose());
    //   timestampedPose.setTimestamp(pose.getTimestamp());
    //   // updateOdometryWithVision(pose.getPose(), pose.getTimestamp());
    // }
    // if (distancePose(pose.getPose(), swerveDrive.getPoseMeters())
    //     < DriveConstants.kUpdateThreshold) {
    //   updateOdometryWithVision(pose.getPose(), pose.getTimestamp());
    // }
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
    swerveDrive.move(vXmps, vYmps, vOmegaRadps, false);
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
  }

  public void setGyroOffset(Rotation2d rotation) {
    swerveDrive.setGyroOffset(rotation);
  }

  public void teleResetGyro() {
    logger.info("Driver Joystick: Reset Gyro");
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0.0));
    swerveDrive.resetGyro();
    swerveDrive.resetOdometry(
        new Pose2d(swerveDrive.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)));
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    logger.info("reset odometry with: {}", pose);
  }

  public Rotation2d getGyroRotation2d() {
    return swerveDrive.getHeading();
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

  public void updateOdometryWithVision(Pose2d calculatedPose, long timestamp) {
    odometryStrategy.addVisionMeasurement(calculatedPose, timestamp);
  }

  // Holonomic Controller
  public void calculateController(State desiredState, Rotation2d desiredAngle) {
    holoContInput = desiredState;
    holoContAngle = desiredAngle;
    holoContOutput = holonomicController.calculate(getPoseMeters(), desiredState, desiredAngle);
    logger.info("input: {}, output: {}", holoContInput, holoContOutput);
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
        new Measure("Gyro Rate", () -> getGyroRate()));
  }
}
