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
    double rawDegs = 90.0 + getPos() / Constants.ShoulderConstants.kTicksPerDeg;

    double a = Constants.ShoulderConstants.kShoulderLen;
    double b = Constants.ShoulderConstants.kShoulderUpperToElevatorUpperPivotDist;
    double c = Constants.ShoulderConstants.kElevatorPivotDist;
    double d = Constants.ShoulderConstants.kShoulderLowerToElevatorLowerPivotDist;
    double f = Constants.ShoulderConstants.kElevatorBaseToPivot;
    double g = Constants.ShoulderConstants.kElevatorBaseToElevatorUpperPivot;

    double e = Math.sqrt(a * a + d * d - 2 * a * d * Math.cos(Math.toRadians(rawDegs)));

    double unadjustedAngle =
        Math.toDegrees(
            Math.acos((d * d + e * e - a * a) / (2 * d * e))
                + Math.acos((e * e + c * c - b * b) / (2 * e * c)));

    return 90.0 - (180.0 - unadjustedAngle - Math.toDegrees(Math.atan2(g, f)));
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
