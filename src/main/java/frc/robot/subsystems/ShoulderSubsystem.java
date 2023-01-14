package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShoulderSubsystem extends MeasurableSubsystem {
  private TalonSRX shoulderTalon;
  private DigitalInput zeroShoulderInput = new DigitalInput(Constants.ShoulderConstants.kZeroId);
  private double desiredPosition;

  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public ShoulderSubsystem() {
    shoulderTalon = new TalonSRX(Constants.ShoulderConstants.kShoulderId);
    shoulderTalon.configAllSettings(Constants.ShoulderConstants.getShoulderTalonConfig());
    shoulderTalon.configSupplyCurrentLimit(
        Constants.ShoulderConstants.getShoulderTalonSupplyLimitConfig());

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
    setPos(Constants.ShoulderConstants.kZeroRads + degs * Constants.ShoulderConstants.kTicksPerDeg);
  }

  public double getPos() {
    return shoulderTalon.getSelectedSensorPosition();
  }

  public boolean isFinished() {
    return Math.abs(desiredPosition - getPos()) <= Constants.ShoulderConstants.kAllowedError;
  }

  public void zeroShoulder() {
    if (zeroShoulderInput.get()) {
      double absolute = shoulderTalon.getSelectedSensorPosition();
      double offset = absolute - Constants.ShoulderConstants.kShoulderZeroTicks;

      shoulderTalon.setSelectedSensorPosition(offset);

      logger.info(
          "Shoulder zeroed; offset: {} zeroTicks: {} absolute: {}",
          offset,
          Constants.ShoulderConstants.kShoulderZeroTicks,
          absolute);

    } else {
      shoulderTalon.configPeakOutputForward(0, 0);
      shoulderTalon.configPeakOutputReverse(0, 0);
      logger.error("Shoulder zero failed. Killing shoulder...");
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(shoulderTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
