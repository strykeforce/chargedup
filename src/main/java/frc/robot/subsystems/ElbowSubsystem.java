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
      percentOutput = {0.3, -0.3, 0.3},
      encoderChange = (int) ElbowConstants.kShelfElbow)
  private TalonFX elbowFalcon;

  private double elbowError = 0;
  private double setPointTicks = 0;
  private CANifier remoteEncoder;
  private Logger logger = LoggerFactory.getLogger(ElbowSubsystem.class);
  private Constants constants;
  private double absoluteTalon = 0.0;
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

  public void setMotionMagic(boolean isAuto) {
    if (isAuto) {
      elbowFalcon.configMotionAcceleration(ElbowConstants.kElbowAutoMotionAcceleration);
      elbowFalcon.configMotionCruiseVelocity(ElbowConstants.kElbowAutoMotionCruiseVelocity);
    } else {
      elbowFalcon.configMotionAcceleration(ElbowConstants.kElbowTeleMotionAcceleration);
      elbowFalcon.configMotionCruiseVelocity(ElbowConstants.kElbowTeleMotionCruiseVelocity);
    }
  }

  @BeforeHealthCheck
  public boolean goToZero() {
    setPos(0.0);
    return isFinished();
  }

  // @AfterHealthCheck
  // public boolean returnToZero() {
  //   setPos(0.0);
  //   return isFinished();
  // }

  private int getPulseWidthFor(PWMChannel channel) {
    double[] pulseWidthandPeriod = new double[2];
    remoteEncoder.getPWMInput(channel, pulseWidthandPeriod);
    return (int) (4096.0 * pulseWidthandPeriod[0] / pulseWidthandPeriod[1]);
  }

  public void zeroElbowStow() {
    rotateOpenLoop(0.0);
    zeroElbow();
  }

  public int printElbowError() {
    int absolute = getPulseWidthFor(PWMChannel.PWMChannel0);
    logger.info(
        "Elbow at zero, absolute: {}, zero ticks: {}, difference: {}",
        absolute,
        Constants.kElbowZeroTicks,
        Math.abs(absolute - Constants.kElbowZeroTicks));
    elbowError = Math.abs(absolute - Constants.kElbowZeroTicks);
    return Math.abs(absolute - Constants.kElbowZeroTicks);
  }

  private void zeroElbow() {
    int absoluteTicks = getPulseWidthFor(PWMChannel.PWMChannel0);
    absoluteTalon = absoluteTicks;
    int offset = absoluteTicks - Constants.kElbowZeroTicks;
    logger.info("Current Elbow Position: {}", elbowFalcon.getSelectedSensorPosition());
    elbowFalcon.setSelectedSensorPosition(offset * Constants.ElbowConstants.kOffsetFactor);
    remoteEncoder.setQuadraturePosition(offset, 10);
    logger.info(
        "Zeroed elbow, absolute: {}, offset: {}, zero ticks: {}",
        absoluteTicks,
        offset,
        Constants.kElbowZeroTicks);
  }

  public double getAbsoluteEncoder() {
    return elbowFalcon.getSelectedSensorPosition();
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
    return Set.of(
        new Measure("Relative Degrees", () -> getRelativeDegs()),
        new Measure("Absolute Ticks USED", () -> absoluteTalon),
        new Measure("The queried values", () -> (double) getPulseWidthFor(PWMChannel.PWMChannel0)), 
        new Measure("Elbow Error", () -> elbowError), new Measure("SelectedSensor /52.4", () -> elbowFalcon.getSelectedSensorPosition()/52.4));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(elbowFalcon);
    telemetryService.register(new CanifierMeasurable(remoteEncoder));
  }
}
