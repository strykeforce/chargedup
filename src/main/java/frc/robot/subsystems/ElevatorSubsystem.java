package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import java.util.Set;
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

public class ElevatorSubsystem extends MeasurableSubsystem implements ArmComponent {
  private static final Logger logger = LoggerFactory.getLogger(ElevatorSubsystem.class);

  @HealthCheck
  @Position(
      percentOutput = {-0.2, 0.2},
      encoderChange = 3_000)
  private TalonFX leftMainFalcon;

  @HealthCheck
  @Follow(leader = ElevatorConstants.kLeftMainId)
  private TalonFX rightFollowFalcon;

  private double desiredPosition;
  private ElevatorState elevatorState;
  private boolean hasZeroed = false;
  private boolean setToGreaterPos = false;

  private int elevatorZeroStableCounts;

  public ElevatorSubsystem() {
    leftMainFalcon = new TalonFX(Constants.ElevatorConstants.kLeftMainId);
    rightFollowFalcon = new TalonFX(Constants.ElevatorConstants.kRightFollowId);
    leftMainFalcon.configFactoryDefault();
    leftMainFalcon.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    leftMainFalcon.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    leftMainFalcon.setNeutralMode(NeutralMode.Brake);
    leftMainFalcon.setInverted(false);

    rightFollowFalcon.configFactoryDefault();
    rightFollowFalcon.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    rightFollowFalcon.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    rightFollowFalcon.setNeutralMode(NeutralMode.Brake);
    rightFollowFalcon.follow(leftMainFalcon);
    rightFollowFalcon.setInverted(true);

    elevatorZeroStableCounts = 0;
    desiredPosition = getPos();
    elevatorState = ElevatorState.IDLE;
  }

  @BeforeHealthCheck
  public boolean goToZero() {
    setPos(ElevatorConstants.kStowElevator);
    return isFinished();
  }

  @AfterHealthCheck
  public boolean returnToZero() {
    setPos(ElevatorConstants.kStowElevator);
    return isFinished();
  }

  public void setPos(double location) {
    if (desiredPosition != location) logger.info("Elevator moving to {}", location);
    setToGreaterPos = location > getPos();
    desiredPosition = location;
    leftMainFalcon.set(TalonFXControlMode.MotionMagic, location);
  }

  public void setPct(double pct) {
    logger.info("Elevator moving at {} speed", pct);
    leftMainFalcon.set(TalonFXControlMode.PercentOutput, pct);
  }

  public double getPos() {
    return leftMainFalcon.getSelectedSensorPosition();
  }

  public double getExtensionMeters() {
    double unadjusted =
        Constants.ElevatorConstants.kMaxExtension
            + getPos() / Constants.ElevatorConstants.kTicksPerMeter;
    return Math.hypot(Constants.ShoulderConstants.kElevatorBaseToPivot, unadjusted);
  }

  public boolean isFinished() {
    return Math.abs(desiredPosition - getPos()) <= Constants.ElevatorConstants.kAllowedError;
  }

  public boolean canStartNextAxis(double canStartTicks) {
    return getPos() >= (canStartTicks - Constants.ElevatorConstants.kAllowedError)
            && setToGreaterPos
        || getPos() <= (canStartTicks + Constants.ElevatorConstants.kAllowedError)
            && !setToGreaterPos;
  }

  public void unReinforceElevator() {
    setPct(0.0);

    leftMainFalcon.configForwardSoftLimitEnable(true);
    leftMainFalcon.configReverseSoftLimitEnable(true);
    // rightFollowFalcon.configForwardSoftLimitEnable(true);
    // rightFollowFalcon.configReverseSoftLimitEnable(true);

    leftMainFalcon.configStatorCurrentLimit(ElevatorConstants.getElevStatorTurnOff());
    rightFollowFalcon.configStatorCurrentLimit(ElevatorConstants.getElevStatorTurnOff());

    logger.info("Elevator is not reinforcing");
  }

  public void reinforceElevator() {
    leftMainFalcon.configForwardSoftLimitEnable(false);
    leftMainFalcon.configReverseSoftLimitEnable(false);
    // rightFollowFalcon.configForwardSoftLimitEnable(false);
    // rightFollowFalcon.configReverseSoftLimitEnable(false);

    leftMainFalcon.configStatorCurrentLimit(
        ElevatorConstants.getElevStatorCurrentLimitConfiguration());
    // rightFollowFalcon.configStatorCurrentLimit(
    //     ElevatorConstants.getElevStatorCurrentLimitConfiguration());

    setPct(-Constants.ElevatorConstants.kElevatorZeroSpeed);
    logger.info("Reinforcing Elevator");
  }

  public void zeroElevator() {
    leftMainFalcon.configForwardSoftLimitEnable(false);
    leftMainFalcon.configReverseSoftLimitEnable(false);
    rightFollowFalcon.configForwardSoftLimitEnable(false);
    rightFollowFalcon.configReverseSoftLimitEnable(false);

    leftMainFalcon.configStatorCurrentLimit(
        ElevatorConstants.getElevStatorCurrentLimitConfiguration());
    rightFollowFalcon.configStatorCurrentLimit(
        ElevatorConstants.getElevStatorCurrentLimitConfiguration());

    setPct(Constants.ElevatorConstants.kElevatorZeroSpeed);

    logger.info("Elevator is zeroing");
    elevatorState = ElevatorState.ZEROING;
  }

  public boolean hasZeroed() {
    return hasZeroed;
  }

  public void setSoftLimits(double minTicks, double maxTicks) {
    leftMainFalcon.configForwardSoftLimitThreshold(maxTicks);
    leftMainFalcon.configReverseSoftLimitThreshold(minTicks);
    // rightFollowFalcon.configForwardSoftLimitThreshold(maxTicks);
    // rightFollowFalcon.configReverseSoftLimitThreshold(minTicks);
  }

  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(leftMainFalcon);
    telemetryService.register(rightFollowFalcon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Extension meters", () -> getExtensionMeters()));
  }

  @Override
  public void periodic() {
    switch (elevatorState) {
      case IDLE:
        break;
      case ZEROING:
        if (Math.abs(leftMainFalcon.getSelectedSensorVelocity())
            < Constants.ElevatorConstants.kZeroTargetSpeedTicksPer100ms) {
          elevatorZeroStableCounts++;
        } else {
          elevatorZeroStableCounts = 0;
        }

        if (elevatorZeroStableCounts > Constants.ElevatorConstants.kZeroStableCounts) {
          setPct(0);
          leftMainFalcon.setSelectedSensorPosition(0.0);
          rightFollowFalcon.setSelectedSensorPosition(0.0);

          setPct(0);
          leftMainFalcon.configStatorCurrentLimit(ElevatorConstants.getElevStatorTurnOff());
          rightFollowFalcon.configStatorCurrentLimit(ElevatorConstants.getElevStatorTurnOff());
          leftMainFalcon.configSupplyCurrentLimit(
              Constants.ElevatorConstants.getElevatorSupplyLimitConfig(),
              Constants.kTalonConfigTimeout);

          rightFollowFalcon.configSupplyCurrentLimit(
              Constants.ElevatorConstants.getElevatorSupplyLimitConfig(),
              Constants.kTalonConfigTimeout);

          leftMainFalcon.configForwardSoftLimitEnable(true);
          leftMainFalcon.configReverseSoftLimitEnable(true);

          desiredPosition = 0;
          elevatorState = ElevatorState.ZEROED;
          logger.info("Elevator is zeroed");
          hasZeroed = true;
          break;
        }
      case ZEROED:
        break;
      default:
        break;
    }
  }

  public enum ElevatorState {
    IDLE,
    ZEROING,
    ZEROED
  }
}
