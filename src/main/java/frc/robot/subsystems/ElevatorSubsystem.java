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
  private TalonFX elevatorFalcon;
  private double desiredPosition;
  private ElevatorState elevatorState;

  private int elevatorZeroStableCounts;

  public ElevatorSubsystem() {
    elevatorFalcon = new TalonFX(Constants.ElevatorConstants.kLeftMainId);
    elevatorFalcon.configFactoryDefault();
    elevatorFalcon.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    elevatorFalcon.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    elevatorFalcon.setNeutralMode(NeutralMode.Brake);

    elevatorZeroStableCounts = 0;
    desiredPosition = getPos();
    elevatorState = ElevatorState.IDLE;
  }

  public void setPos(double location) {
    logger.info("Elevator moving to {}", location);
    desiredPosition = location;
    elevatorFalcon.set(TalonFXControlMode.MotionMagic, location);
  }

  public void setPct(double pct) {
    logger.info("Elevator moving at {}% speed", pct);
    elevatorFalcon.set(TalonFXControlMode.PercentOutput, pct);
  }

  public double getPos() {
    return elevatorFalcon.getSelectedSensorPosition();
  }

  public double getExtensionMeters() {
    return Constants.ElevatorConstants.kMaxExtension
        + 2 * getPos() / Constants.ElevatorConstants.kTicksPerMeter;
  }

  public boolean isFinished() {
    return Math.abs(desiredPosition - getPos()) <= Constants.ElevatorConstants.kAllowedError;
  }

  public void zeroElevator() {
    elevatorFalcon.configForwardSoftLimitEnable(false);
    elevatorFalcon.configReverseSoftLimitEnable(false);

    elevatorFalcon.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorZeroSupplyCurrentLimit(),
        Constants.kTalonConfigTimeout);

    setPct(Constants.ElevatorConstants.kElevatorZeroSpeed);

    logger.info("Elevator is zeroing");
    elevatorState = ElevatorState.ZEROING;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(elevatorFalcon);
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
        if (Math.abs(elevatorFalcon.getSelectedSensorVelocity())
            < Constants.ElevatorConstants.kZeroTargetSpeedTicksPer100ms) {
          elevatorZeroStableCounts++;
        } else {
          elevatorZeroStableCounts = 0;
        }

        if (elevatorZeroStableCounts > Constants.ElevatorConstants.kZeroStableCounts) {
          elevatorFalcon.setSelectedSensorPosition(0.0);
          elevatorFalcon.configSupplyCurrentLimit(
              Constants.ElevatorConstants.getElevatorSupplyLimitConfig(),
              Constants.kTalonConfigTimeout);

          elevatorFalcon.configForwardSoftLimitEnable(true);
          elevatorFalcon.configReverseSoftLimitEnable(true);

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
