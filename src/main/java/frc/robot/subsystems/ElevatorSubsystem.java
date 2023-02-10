package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElevatorSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(ElevatorSubsystem.class);
  private TalonFX leftMainFalcon;
  private TalonFX rightFollowFalcon;
  private double desiredPosition;
  private ElevatorState elevatorState;

  private int elevatorZeroStableCounts;

  public ElevatorSubsystem() {
    leftMainFalcon = new TalonFX(Constants.ElevatorConstants.kLeftMainId);
    rightFollowFalcon = new TalonFX(Constants.ElevatorConstants.kRightFollowId);
    leftMainFalcon.configFactoryDefault();
    leftMainFalcon.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    leftMainFalcon.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    leftMainFalcon.setNeutralMode(NeutralMode.Brake);
    rightFollowFalcon.configFactoryDefault();
    rightFollowFalcon.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    rightFollowFalcon.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    rightFollowFalcon.setNeutralMode(NeutralMode.Brake);
    rightFollowFalcon.follow(leftMainFalcon);

    elevatorZeroStableCounts = 0;
    desiredPosition = getPos();
    elevatorState = ElevatorState.IDLE;
  }

  public void setPos(double location) {
    logger.info("Elevator moving to {}", location);
    desiredPosition = location;
    leftMainFalcon.set(TalonFXControlMode.MotionMagic, location);
  }

  public void setPct(double pct) {
    logger.info("Elevator moving at {}% speed", pct);
    leftMainFalcon.set(TalonFXControlMode.PercentOutput, pct);
  }

  public double getPos() {
    return leftMainFalcon.getSelectedSensorPosition();
  }

  public boolean isFinished() {
    return Math.abs(desiredPosition - getPos()) <= Constants.ElevatorConstants.kAllowedError;
  }

  public void zeroElevator() {
    leftMainFalcon.configForwardSoftLimitEnable(false);
    leftMainFalcon.configReverseSoftLimitEnable(false);

    leftMainFalcon.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorZeroSupplyCurrentLimit(),
        Constants.kTalonConfigTimeout);

    setPct(Constants.ElevatorConstants.kElevatorZeroSpeed);

    logger.info("Elevator is zeroing");
    elevatorState = ElevatorState.ZEROING;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(leftMainFalcon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
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
          leftMainFalcon.setSelectedSensorPosition(0.0);
          leftMainFalcon.configSupplyCurrentLimit(
              Constants.ElevatorConstants.getElevatorSupplyLimitConfig(),
              Constants.kTalonConfigTimeout);

          leftMainFalcon.configForwardSoftLimitEnable(true);
          leftMainFalcon.configReverseSoftLimitEnable(true);

          setPct(0);
          desiredPosition = 0;
          elevatorState = ElevatorState.ZEROED;
          break;
        }
      case ZEROED:
        logger.info("Elevator is zeroed");
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
