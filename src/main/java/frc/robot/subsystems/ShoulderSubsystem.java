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

public class ShoulderSubsystem extends MeasurableSubsystem {
  private TalonSRX shoulderTalon;
  private double desiredPosition;

  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public ShoulderSubsystem() {
    shoulderTalon = new TalonSRX(Constants.ShoulderConstants.kShoulderId);
    shoulderTalon.configAllSettings(Constants.ShoulderConstants.getShoulderTalonConfig());
    shoulderTalon.configSupplyCurrentLimit(
        Constants.ShoulderConstants.getShoulderTalonSupplyLimitConfig());
    zeroShoulder();
    desiredPosition = getPos();
  }

  public void setPos(double location) {
    desiredPosition = location;
    shoulderTalon.set(ControlMode.MotionMagic, location);
  }

  public void setPct(double pct) {
    shoulderTalon.set(ControlMode.PercentOutput, pct);
  }

  public void setDegs(double degs) {
    setPos(Constants.ShoulderConstants.kZeroDegs + degs * Constants.ShoulderConstants.kTicksPerDeg);
  }

  public double getDegs() {
    return getPos() / Constants.ShoulderConstants.kTicksPerDeg;
  }

  public double getPos() {
    return shoulderTalon.getSelectedSensorPosition();
  }

  public boolean isFinished() {
    return Math.abs(desiredPosition - getPos()) <= Constants.ShoulderConstants.kAllowedError;
  }

  public void zeroShoulder() {
    double absolute = shoulderTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    double offset = absolute - Constants.ShoulderConstants.kShoulderZeroTicks;
    shoulderTalon.setSelectedSensorPosition(offset);
    logger.info(
        "Absolute: {}, Zero pos: {}, Offset: {}",
        absolute,
        Constants.ShoulderConstants.kShoulderZeroTicks,
        offset);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(shoulderTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Degrees", () -> getDegs()));
  }
}
