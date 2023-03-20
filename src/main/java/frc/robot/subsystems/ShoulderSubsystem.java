package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;
import java.util.Set;

import javax.swing.event.ChangeListener;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.AfterHealthCheck;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.Follow;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Position;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShoulderSubsystem extends MeasurableSubsystem implements ArmComponent {

  @HealthCheck
  @Position(
      percentOutput = {0.2, -0.2},
      encoderChange = 1000)
  private TalonSRX leftMainShoulderTalon;

  @HealthCheck
  @Follow(leader = ShoulderConstants.kShoulderId)
  private TalonSRX rightFollowerShoulderTalon;

  private double desiredPosition;
  private Constants constants;
  private boolean setToGreaterPos = false;
  private double leftTwist;
  private double rightTwist;

  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public ShoulderSubsystem(Constants constants) {
    this.constants = constants;
    leftMainShoulderTalon = new TalonSRX(Constants.ShoulderConstants.kShoulderId);
    rightFollowerShoulderTalon = new TalonSRX(Constants.ShoulderConstants.kFollowerShoulderId);
    leftMainShoulderTalon.configFactoryDefault();
    leftMainShoulderTalon.configAllSettings(Constants.ShoulderConstants.getShoulderTalonConfig());
    leftMainShoulderTalon.configSupplyCurrentLimit(
        Constants.ShoulderConstants.getShoulderTalonSupplyLimitConfig());
    leftMainShoulderTalon.setNeutralMode(NeutralMode.Brake);

    rightFollowerShoulderTalon.configFactoryDefault();
    rightFollowerShoulderTalon.configAllSettings(
        Constants.ShoulderConstants.getShoulderTalonConfig());
    rightFollowerShoulderTalon.configSupplyCurrentLimit(
        Constants.ShoulderConstants.getShoulderTalonSupplyLimitConfig());
    rightFollowerShoulderTalon.setNeutralMode(NeutralMode.Brake);
    rightFollowerShoulderTalon.setInverted(true);

    zeroShoulder();
    desiredPosition = getPos();
  }

  @BeforeHealthCheck
  public boolean goToZero() {
    setPos(0.0);
    return isFinished();
  }

  @AfterHealthCheck
  public boolean returnToZero() {
    setPos(0.0);
    return isFinished();
  }

  public void setPos(double location) {
    if (location != desiredPosition) logger.info("Moving Shoulder to: {}", location);
    setToGreaterPos = location > getPos();
    desiredPosition = location;
    leftMainShoulderTalon.set(ControlMode.MotionMagic, location);
    rightFollowerShoulderTalon.set(ControlMode.MotionMagic, location);
  }

  public void setPct(double pct) {
    rightFollowerShoulderTalon.set(ControlMode.PercentOutput, pct);
    leftMainShoulderTalon.set(ControlMode.PercentOutput, pct);
  }

  public void setDegs(double degs) {
    setPos(Constants.ShoulderConstants.kZeroDegs + degs * Constants.ShoulderConstants.kTicksPerDeg);
  }

  public double getShoulderAngle() {
    return 90.0 + getPos() / Constants.ShoulderConstants.kTicksPerDeg;
  }

  public double getDegs() {
    return 90.0 - (getPos() / Constants.ShoulderConstants.kTicksPerDeg);
  }

  public double getPos() {
    return leftMainShoulderTalon.getSelectedSensorPosition();
  }

  public boolean isFinished() {
    return Math.abs(desiredPosition - getPos()) <= Constants.ShoulderConstants.kAllowedError;
  }

  public boolean canStartNextAxis(double canStartTicks) {
    return getPos() >= (canStartTicks - Constants.ShoulderConstants.kAllowedError)
            && setToGreaterPos
        || getPos() <= (canStartTicks + Constants.ShoulderConstants.kAllowedError)
            && !setToGreaterPos;
  }

  public void unTwist(double origin) {
    rightFollowerShoulderTalon.set(ControlMode.MotionMagic, origin);
    leftMainShoulderTalon.set(ControlMode.MotionMagic, origin);
  }

  public void setTwist(double origin) {
    rightTwist = origin;
    leftTwist = origin;
  }

  public void twistShoulder(double change) {
    leftMainShoulderTalon.set(
        ControlMode.MotionMagic, leftTwist + change);
    rightFollowerShoulderTalon.set(
        ControlMode.MotionMagic, rightTwist - change);
  }

  public void zeroShoulder() {
    double absoluteMain =
        leftMainShoulderTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    double offsetMain = absoluteMain - constants.kShoulderMainZeroTicks;

    double absoluteFollower =
        rightFollowerShoulderTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    double offsetFollower = absoluteFollower - constants.kShoulderFollowerZeroTicks;
    leftMainShoulderTalon.setSelectedSensorPosition(offsetMain);
    rightFollowerShoulderTalon.setSelectedSensorPosition(offsetFollower);

    logger.info(
        "Absolute Main: {}, Zero pos Main: {}, Offset Main: {}",
        absoluteMain,
        constants.kShoulderMainZeroTicks,
        offsetMain);
    logger.info(
        "Absolute Follower: {}, Zero pos Follower: {}, Offset Follower: {}",
        absoluteFollower,
        constants.kShoulderFollowerZeroTicks,
        offsetFollower);
  }

  public void setSoftLimits(double minTicks, double maxTicks) {
    leftMainShoulderTalon.configForwardSoftLimitThreshold(maxTicks);
    leftMainShoulderTalon.configReverseSoftLimitThreshold(minTicks);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(leftMainShoulderTalon);
    telemetryService.register(rightFollowerShoulderTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Degrees", () -> getDegs()),
        new Measure("Shoulder Degrees", () -> getShoulderAngle()),
        new Measure("Shoulder Tick Position", () -> getPos()));
  }
}
