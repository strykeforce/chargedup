package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class HandSubsystem extends MeasurableSubsystem {
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private TalonSRX handTalon;
  private TalonSRX wristTalon;
  private double desiredPosition;
  private HandState handState;
  private int handZeroStableCounts;

  public HandSubsystem() {
    handTalon = new TalonSRX(Constants.HandConstants.kHandTalonId);
    handTalon.configAllSettings(Constants.HandConstants.getHandTalonConfig());
    handTalon.configSupplyCurrentLimit(Constants.HandConstants.getHandSupplyLimitConfig());

    wristTalon = new TalonSRX(Constants.HandConstants.kWristTalonId); // DOES NOTHING
    wristTalon.configAllSettings(Constants.HandConstants.getHandTalonConfig());
    wristTalon.configSupplyCurrentLimit(Constants.HandConstants.getHandSupplyLimitConfig());

    desiredPosition = getPos();
    handState = HandState.IDLE;
    handZeroStableCounts = 0;
  }

  public void setPos(double location) {
    logger.info("Hand moving to {}", location);
    desiredPosition = location;
    handTalon.set(ControlMode.MotionMagic, location);
  }

  public void setPct(double pct) {
    logger.info("Hand speed: {}", pct);
    handTalon.set(ControlMode.PercentOutput, pct);
  }

  public double getPos() {
    return handTalon.getSelectedSensorPosition();
  }

  public boolean isFinished() {
    return Math.abs(desiredPosition - getPos()) <= Constants.HandConstants.kAllowedError;
  }

  public void zeroHand() {
    handTalon.configForwardSoftLimitEnable(false);

    handTalon.configSupplyCurrentLimit(
        Constants.HandConstants.getHandZeroSupplyCurrentLimit(), Constants.kTalonConfigTimeout);

    setPct(Constants.HandConstants.kHandZeroSpeed);

    logger.info("Hand is zeroing");
    handState = HandState.ZEROING;
  }

  public void grabCube() {
    logger.info("Grabbing cube");
    setPos(Constants.HandConstants.kCubeGrabbingPosition);
  }

  public void grabCone() {
    logger.info("Grabbing cone");
    setPos(Constants.HandConstants.kConeGrabbingPosition);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(handTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  @Override
  public void periodic() {
    switch (handState) {
      case IDLE:
        break;
      case ZEROING:
        if (Math.abs(handTalon.getSelectedSensorVelocity())
            < Constants.HandConstants.kZeroTargetSpeedTicksPer100ms) {
          handZeroStableCounts++;
        } else {
          handZeroStableCounts = 0;
        }

        if (handZeroStableCounts > Constants.HandConstants.kZeroStableCounts) {
          handTalon.setSelectedSensorPosition(0.0);
          handTalon.configSupplyCurrentLimit(
              Constants.HandConstants.getHandSupplyLimitConfig(), Constants.kTalonConfigTimeout);

          handTalon.configForwardSoftLimitEnable(true);
          handTalon.configReverseSoftLimitEnable(true);

          setPct(0);
          desiredPosition = 0;
          handState = HandState.ZEROED;
          break;
        }
      case ZEROED:
        logger.info("Hand is zeroed");
        break;
      default:
        break;
    }
  }

  public enum HandState {
    IDLE,
    ZEROING,
    ZEROED
  }
}
