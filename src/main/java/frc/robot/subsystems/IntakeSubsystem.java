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
import org.strykeforce.healthcheck.HealthCheck;
import org.strykeforce.healthcheck.Timed;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class IntakeSubsystem extends MeasurableSubsystem {
  private IntakeState currIntakeState = IntakeState.RETRACTED;

  @HealthCheck
  @Timed(
      percentOutput = {0.5, -0.5},
      duration = 5)
  private TalonFX intakeFalcon;

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

  public void intakeOpenLoop(double percentOutput) {
    intakeFalcon.set(ControlMode.PercentOutput, percentOutput);
  }

  public void extendClosedLoop() {
    extendTalon.set(ControlMode.MotionMagic, IntakeConstants.kExtendPosTicks);
    intakeSetPointTicks = IntakeConstants.kExtendPosTicks;
    isIntakeExtended = true;
    // logger.info("Intake is extending to {}", IntakeConstants.kExtendPosTicks);
  }

  public void retractClosedLoop(double retractPos) {
    extendTalon.set(ControlMode.MotionMagic, retractPos);
    intakeSetPointTicks = retractPos;
    isIntakeExtended = false;
    // logger.info("Intake is retracting to {}", IntakeConstants.kRetractPosTicks);
  }

  public boolean isFinished() {
    return Math.abs(intakeSetPointTicks - extendTalon.getSelectedSensorPosition())
        < IntakeConstants.kCloseEnoughTicks;
  }

  public boolean getIsIntakeExtended() {
    return isIntakeExtended;
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

  public void retractIntake() {
    logger.info("Retract Intake to: {}", IntakeConstants.kRetractPosTicks);
    currIntakeState = IntakeState.RETRACTED;
    intakeOpenLoop(0);
    retractClosedLoop(IntakeConstants.kRetractPosTicks);
  }

  public void retractToPickupFromIntake() {
    logger.info("Retract Intake to Pickup Pos: {}", IntakeConstants.kPickupPosTicks);
    currIntakeState = IntakeState.RETRACTED;
    intakeOpenLoop(0);
    retractClosedLoop(IntakeConstants.kPickupPosTicks);
  }

  public void startIntaking(boolean doHolding) {
    logger.info("Start intaking");
    this.doHolding = doHolding;
    currIntakeState = IntakeState.INTAKING;
    intakeOpenLoop(IntakeConstants.kIntakeSpeed);
    extendClosedLoop();
  }

  public void startIntaking() {
    startIntaking(false);
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
