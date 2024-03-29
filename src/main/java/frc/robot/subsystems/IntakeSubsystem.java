package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.AfterHealthCheck;
import org.strykeforce.healthcheck.BeforeHealthCheck;
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Position;
import org.strykeforce.healthcheck.Timed;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class IntakeSubsystem extends MeasurableSubsystem {
  private IntakeState currIntakeState = IntakeState.RETRACTED;
  private boolean setToGreaterPos = false;

  @HealthCheck(order = 1)
  @Timed(
      percentOutput = {0.5, -0.5},
      duration = 3)
  private TalonFX intakeFalcon;

  @HealthCheck(order = 2)
  @Position(
      percentOutput = {-0.5, 0.5},
      encoderChange = 2_000)
  private TalonSRX extendTalon;

  private double intakeSetPointTicks;
  private boolean isIntakeExtended = false;
  private Timer ejectTimer = new Timer();
  private boolean doHolding = false;
  private final Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);
  private int beamBreakStableCounts = 0;
  private boolean beamBroken = false;
  private Constants constants;

  public IntakeSubsystem(Constants constants) {
    this.constants = constants;
    intakeFalcon = new TalonFX(IntakeConstants.kIntakeFalconID);
    intakeFalcon.configFactoryDefault();
    intakeFalcon.configAllSettings(IntakeConstants.getIntakeFalconConfig());
    intakeFalcon.setNeutralMode(NeutralMode.Coast);

    extendTalon = new TalonSRX(IntakeConstants.kExtendTalonID);
    extendTalon.configFactoryDefault();
    extendTalon.configAllSettings(IntakeConstants.getExtendTalonConfig());
    extendTalon.configSupplyCurrentLimit(IntakeConstants.getTalonSupplyLimitConfig());
    extendTalon.configForwardLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
    extendTalon.setNeutralMode(NeutralMode.Brake);

    zeroIntake();
  }

  @BeforeHealthCheck
  public boolean goToZero() {
    retractIntake();
    return isFinished();
  }

  @AfterHealthCheck
  public boolean returnToZero() {
    retractIntake();
    return isFinished();
  }

  public void intakeOpenLoop(double percentOutput) {
    intakeFalcon.set(ControlMode.PercentOutput, percentOutput);
  }

  public void extendClosedLoop() {
    extendTalon.set(ControlMode.MotionMagic, Constants.kExtendPosTicks);
    intakeSetPointTicks = Constants.kExtendPosTicks;
    setToGreaterPos = intakeSetPointTicks > getPos();
    isIntakeExtended = true;
    // logger.info("Intake is extending to {}", IntakeConstants.kExtendPosTicks);
  }

  public double getPos() {
    return extendTalon.getSelectedSensorPosition();
  }

  public void retractClosedLoop(double retractPos) {
    extendTalon.set(ControlMode.MotionMagic, retractPos);
    intakeSetPointTicks = retractPos;
    isIntakeExtended = false;
    setToGreaterPos = intakeSetPointTicks > getPos();
    // logger.info("Intake is retracting to {}", IntakeConstants.kRetractPosTicks);
  }

  public boolean isFinished() {
    return Math.abs(intakeSetPointTicks - extendTalon.getSelectedSensorPosition())
        < IntakeConstants.kCloseEnoughTicks;
  }

  public boolean getIsIntakeExtended() {
    return isIntakeExtended;
  }

  public boolean canStartNextAxis(double canStartTicks) {
    return getPos() >= (canStartTicks - Constants.IntakeConstants.kAllowedError) && setToGreaterPos
        || getPos() <= (canStartTicks + Constants.IntakeConstants.kAllowedError)
            && !setToGreaterPos;
  }

  public void zeroIntake() {
    int absPos = extendTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;

    int offset = absPos - constants.kIntakeZeroTicks;
    extendTalon.setSelectedSensorPosition(offset);
    logger.info(
        "Intake zeroed; offset: {} zeroTicks: {} absPosition: {}",
        offset,
        constants.kIntakeZeroTicks,
        absPos);
  }

  public void retractIntake(boolean rollersOff) {
    // logger.info("Retract Intake to: {}", IntakeConstants.kRetractPosTicks);
    currIntakeState = IntakeState.RETRACTED;
    if (rollersOff) intakeOpenLoop(0);
    setToGreaterPos = IntakeConstants.kRetractPosTicks > getPos();
    retractClosedLoop(IntakeConstants.kRetractPosTicks);
  }

  public void retractIntake() {
    retractIntake(true);
  }

  public void retractToPickupFromIntake() {
    logger.info("Retract Intake to Pickup Pos: {}", IntakeConstants.kPickupPosTicks);
    currIntakeState = IntakeState.RETRACTED;
    intakeOpenLoop(0);
    setToGreaterPos = IntakeConstants.kPickupPosTicks > getPos();
    retractClosedLoop(IntakeConstants.kPickupPosTicks);
  }

  public void startIntaking(boolean isAuton) {
    logger.info("Start intaking");
    // this.doHolding = isAuton;
    currIntakeState = IntakeState.INTAKING;
    if (isAuton) intakeOpenLoop(IntakeConstants.kIntakeSpeed);
    extendClosedLoop();
  }

  public void startIntaking() {
    startIntaking(true);
  }

  public void startEjecting() {
    logger.info("Intake ejecting");
    currIntakeState = IntakeState.EJECTING;
    intakeOpenLoop(IntakeConstants.kIntakeEjectSpeed);
    extendClosedLoop();
    ejectTimer.reset();
    ejectTimer.start();
  }

  public IntakeState getState() {
    return currIntakeState;
  }

  public boolean isBeamBroken() {
    if (extendTalon.isFwdLimitSwitchClosed() > 0) beamBreakStableCounts++;
    else beamBreakStableCounts = 0;
    beamBroken = beamBreakStableCounts > IntakeConstants.kBeamBreakStableCounts;
    return beamBreakStableCounts > IntakeConstants.kBeamBreakStableCounts;
  }

  @Override
  public void periodic() {

    switch (currIntakeState) {
      case RETRACTED:
        // intake retracted, falcon off
        break;
      case INTAKING:
        // if we want to hold, check for beam break
        if (doHolding && isBeamBroken()) {
          logger.info("INTAKING -> HOLDING");
          intakeOpenLoop(0);
          currIntakeState = IntakeState.HOLDING;
        }
        break;
      case EJECTING:
        if (ejectTimer.hasElapsed(IntakeConstants.kEjectTimerDelaySec)) {
          ejectTimer.stop();
          startIntaking();
        }
        break;
      case HOLDING:
        // intake extended, falcon off
        break;
      default:
        break;
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Beam Broken", () -> beamBroken ? 1.0 : 0.0));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(extendTalon);
    telemetryService.register(intakeFalcon);
  }

  public enum IntakeState {
    RETRACTED,
    INTAKING,
    EJECTING,
    HOLDING;
  }
}
