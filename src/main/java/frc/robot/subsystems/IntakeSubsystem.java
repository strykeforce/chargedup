package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeState currIntakeState = IntakeState.RETRACTED;
  private TalonFX intakeFalcon;
  private TalonSRX extendTalon;
  private double intakeSetPointTicks;
  private boolean isIntakeExtended = false;
  private Timer ejectTimer = new Timer();
  private int lastAbsPos = Integer.MAX_VALUE;
  private boolean doHolding = false;
  private final Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);

  public IntakeSubsystem() {
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

  public void retractClosedLoop() {
    extendTalon.set(ControlMode.MotionMagic, IntakeConstants.kRetractPosTicks);
    intakeSetPointTicks = IntakeConstants.kRetractPosTicks;
    isIntakeExtended = false;
    // logger.info("Intake is retracting to {}", IntakeConstants.kRetractPosTicks);
  }

  public boolean isIntakeAtPos() {
    return Math.abs(intakeSetPointTicks - extendTalon.getSelectedSensorPosition())
        < IntakeConstants.kCloseEnoughTicks;
  }

  public boolean getIsIntakeExtended() {
    return isIntakeExtended;
  }

  public void zeroIntake() {
    int absPos = extendTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;

    int offset = absPos - IntakeConstants.kIntakeZeroTicks;
    extendTalon.setSelectedSensorPosition(offset);
    logger.info(
        "Intake zeroed; offset: {} zeroTicks: {} absPosition: {}",
        offset,
        IntakeConstants.kIntakeZeroTicks,
        absPos);
  }

  public void retractIntake() {
    currIntakeState = IntakeState.RETRACTED;
    intakeOpenLoop(0);
    retractClosedLoop();
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

  @Override
  public void periodic() {

    switch (currIntakeState) {
      case RETRACTED:
        // intake retracted, falcon off
        break;
      case INTAKING:
        // if we want to hold, check for beam break
        if (doHolding && extendTalon.isFwdLimitSwitchClosed() == 1) {
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

  public enum IntakeState {
    RETRACTED,
    INTAKING,
    EJECTING,
    HOLDING;
  }
}
