package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.ElbowConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.CanifierMeasurable;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElbowSubsystem extends MeasurableSubsystem implements ArmComponent {
  private TalonFX elbowFalcon;
  private TalonFX elbowFollowerFalcon;
  private double setPointTicks = 0;
  private CANifier remoteEncoder;
  private Logger logger = LoggerFactory.getLogger(ElbowSubsystem.class);

  public ElbowSubsystem() {
    elbowFalcon = new TalonFX(ElbowConstants.kElbowFalconID);
    elbowFalcon.configFactoryDefault();
    elbowFalcon.configAllSettings(ElbowConstants.getElbowFalconConfig());

    elbowFollowerFalcon = new TalonFX(ElbowConstants.kElbowFollowerFalconID);
    elbowFollowerFalcon.configFactoryDefault();
    elbowFollowerFalcon.configAllSettings(ElbowConstants.getElbowFalconConfig());

    remoteEncoder = new CANifier(ElbowConstants.kRemoteEncoderID);

    elbowFalcon.configForwardSoftLimitThreshold(ElbowConstants.kForwardSoftLimit);
    elbowFalcon.configForwardSoftLimitEnable(true);
    elbowFalcon.configReverseSoftLimitThreshold(ElbowConstants.kReverseSoftLimit);
    elbowFalcon.configReverseSoftLimitEnable(true);

    elbowFollowerFalcon.configForwardSoftLimitThreshold(ElbowConstants.kForwardSoftLimit);
    elbowFollowerFalcon.configForwardSoftLimitEnable(true);
    elbowFollowerFalcon.configReverseSoftLimitThreshold(ElbowConstants.kReverseSoftLimit);
    elbowFollowerFalcon.configReverseSoftLimitEnable(true);

    elbowFalcon.setNeutralMode(NeutralMode.Brake);
    elbowFollowerFalcon.setNeutralMode(NeutralMode.Brake);

    elbowFollowerFalcon.setInverted(true);
    elbowFollowerFalcon.follow(elbowFalcon);

    zeroElbow();
  }

  private int getPulseWidthFor(PWMChannel channel) {
    double[] pulseWidthandPeriod = new double[2];
    remoteEncoder.getPWMInput(channel, pulseWidthandPeriod);
    return (int) pulseWidthandPeriod[0];
  }

  private void zeroElbow() {
    int absoluteTicks = getPulseWidthFor(PWMChannel.PWMChannel0);
    int offset = absoluteTicks - ElbowConstants.kZeroTicks;
    elbowFalcon.setSelectedSensorPosition(offset * Constants.ElbowConstants.kOffsetFactor);
    remoteEncoder.setQuadraturePosition(offset, 10);
    logger.info(
        "Zeroed elbow, absolute: {}, offset: {}, zero ticks: {}",
        absoluteTicks,
        offset,
        ElbowConstants.kZeroTicks);
  }

  public void rotateOpenLoop(double percentOutput) {
    elbowFalcon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("elbow openloop percentOutput: {}", percentOutput);
  }

  public void setPos(double posTicks) {
    elbowFalcon.set(ControlMode.MotionMagic, posTicks);
    setPointTicks = posTicks;
  }

  public double getRelativeDegs() {
    return ElbowConstants.kZeroDegs
        + remoteEncoder.getQuadraturePosition() / ElbowConstants.kTicksPerDeg;
  }

  public boolean isElbowAtPos() {
    return Math.abs(setPointTicks - elbowFalcon.getSelectedSensorPosition())
        < ElbowConstants.kCloseEnoughTicks;
  }

  @Override
  public void periodic() {
    // nothing here yet :)
  }

  public double getPos() {
    return elbowFalcon.getSelectedSensorPosition();
  }

  public boolean isFinished() {
    return Math.abs(setPointTicks - getPos()) <= Constants.ElbowConstants.kCloseEnoughTicks;
  }

  public void setSoftLimits(double minTicks, double maxTicks) {
    elbowFalcon.configForwardSoftLimitThreshold(maxTicks);
    elbowFalcon.configReverseSoftLimitThreshold(minTicks);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Relative Degrees", () -> getRelativeDegs()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(elbowFalcon);
    telemetryService.register(new CanifierMeasurable(remoteEncoder));
  }
}
