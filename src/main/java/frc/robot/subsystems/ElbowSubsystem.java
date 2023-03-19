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
import org.strykeforce.healthcheck.AfterHealthCheck;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Position;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.CanifierMeasurable;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElbowSubsystem extends MeasurableSubsystem implements ArmComponent {

  @HealthCheck
  @Position(
      percentOutput = {0.1, -0.1},
      encoderChange = 5000)
  private TalonFX elbowFalcon;

  private double setPointTicks = 0;
  private CANifier remoteEncoder;
  private Logger logger = LoggerFactory.getLogger(ElbowSubsystem.class);
  private Constants constants;
  private boolean setToGreaterPos = false;

  public ElbowSubsystem(Constants constants) {
    this.constants = constants;
    elbowFalcon = new TalonFX(ElbowConstants.kElbowFalconID);
    elbowFalcon.configFactoryDefault();
    elbowFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());

    remoteEncoder = new CANifier(ElbowConstants.kRemoteEncoderID);

    elbowFalcon.configForwardSoftLimitThreshold(ElbowConstants.kForwardSoftLimit);
    elbowFalcon.configForwardSoftLimitEnable(true);
    elbowFalcon.configReverseSoftLimitThreshold(ElbowConstants.kReverseSoftLimit);
    elbowFalcon.configReverseSoftLimitEnable(true);

    elbowFalcon.setNeutralMode(NeutralMode.Brake);

    zeroElbow();
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

  private int getPulseWidthFor(PWMChannel channel) {
    double[] pulseWidthandPeriod = new double[2];
    remoteEncoder.getPWMInput(channel, pulseWidthandPeriod);
    return (int) pulseWidthandPeriod[0];
  }

  public void zeroElbowStow() {
    rotateOpenLoop(0.0);
    zeroElbow();
  }

  private void zeroElbow() {
    int absoluteTicks = getPulseWidthFor(PWMChannel.PWMChannel0);
    int offset = absoluteTicks - constants.kElbowZeroTicks;
    logger.info("Current Elbow Position: {}", elbowFalcon.getSelectedSensorPosition());
    elbowFalcon.setSelectedSensorPosition(offset * Constants.ElbowConstants.kOffsetFactor);
    remoteEncoder.setQuadraturePosition(offset, 10);
    logger.info(
        "Zeroed elbow, absolute: {}, offset: {}, zero ticks: {}",
        absoluteTicks,
        offset,
        constants.kElbowZeroTicks);
  }

  public void rotateOpenLoop(double percentOutput) {
    elbowFalcon.set(ControlMode.PercentOutput, percentOutput);
    logger.info("elbow openloop percentOutput: {}", percentOutput);
  }

  public void setPos(double posTicks) {
    if (setPointTicks != posTicks) logger.info("Moving Elbow to: {}", posTicks);
    setToGreaterPos = posTicks > getPos();
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

  public boolean canStartNextAxis(double canStartTicks) {
    return getPos() >= (canStartTicks - Constants.ElbowConstants.kCloseEnoughTicks)
            && setToGreaterPos
        || getPos() <= (canStartTicks + Constants.ElbowConstants.kCloseEnoughTicks)
            && !setToGreaterPos;
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
