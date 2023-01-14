package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
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
  private TalonFX leftMain, rightFollower;
  private double desiredPosition;
  private ElevatorState elevatorState;

  private int elevatorZeroStableCounts;

  public ElevatorSubsystem() {
    leftMain = new TalonFX(Constants.ElevatorConstants.kLeftMainId);
    rightFollower = new TalonFX(Constants.ElevatorConstants.kRightFollowerId);

    leftMain.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    leftMain.configSupplyCurrentLimit(Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    leftMain.setInverted(TalonFXInvertType.Clockwise);

    rightFollower.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    rightFollower.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    rightFollower.setInverted(TalonFXInvertType.CounterClockwise);
    rightFollower.follow(leftMain, FollowerType.PercentOutput);

    elevatorZeroStableCounts = 0;
    desiredPosition = getPos();
    elevatorState = ElevatorState.IDLE;
  }

  public void setPos(double location) {
    logger.info("Elevator moving to {}", location);
    desiredPosition = location;
    leftMain.set(TalonFXControlMode.MotionMagic, location);
  }

  public void setPct(double pct) {
    logger.info("Elevator moving at {}% speed", pct);
    leftMain.set(TalonFXControlMode.PercentOutput, pct);
  }

  public double getPos() {
    return leftMain.getSelectedSensorPosition();
  }

  public boolean isFinished() {
    return Math.abs(desiredPosition - getPos()) <= Constants.ElevatorConstants.kAllowedError;
  }

  public void zeroElevator() {
    leftMain.configForwardSoftLimitEnable(false);
    leftMain.configReverseSoftLimitEnable(false);

    rightFollower.configForwardSoftLimitEnable(false);
    rightFollower.configReverseSoftLimitEnable(false);

    leftMain.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorZeroSupplyCurrentLimit(),
        Constants.kTalonConfigTimeout);
    rightFollower.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorZeroSupplyCurrentLimit(),
        Constants.kTalonConfigTimeout);

    setPct(Constants.ElevatorConstants.kElevatorZeroSpeed);

    logger.info("Elevator is zeroing");
    elevatorState = ElevatorState.ZEROING;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(leftMain);
    telemetryService.register(rightFollower);
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
        if (Math.abs(leftMain.getSelectedSensorVelocity())
            < Constants.ElevatorConstants.kZeroTargetSpeedTicksPer100ms) {
          elevatorZeroStableCounts++;
        } else {
          elevatorZeroStableCounts = 0;
        }

        if (elevatorZeroStableCounts > Constants.ElevatorConstants.kZeroStableCounts) {
          leftMain.setSelectedSensorPosition(0.0);
          rightFollower.setSelectedSensorPosition(0.0);
          leftMain.configSupplyCurrentLimit(
              Constants.ElevatorConstants.getElevatorSupplyLimitConfig(),
              Constants.kTalonConfigTimeout);
          rightFollower.configSupplyCurrentLimit(
              Constants.ElevatorConstants.getElevatorSupplyLimitConfig(),
              Constants.kTalonConfigTimeout);

          leftMain.configForwardSoftLimitEnable(true);
          leftMain.configReverseSoftLimitEnable(true);

          rightFollower.configForwardSoftLimitEnable(true);
          rightFollower.configReverseSoftLimitEnable(true);

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
