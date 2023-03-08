package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.HandConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class HandSubsystem extends MeasurableSubsystem {
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private TalonSRX handLeftTalon;
  private TalonSRX rollerTalon;
  // private TalonSRX handRightTalon;

  private double leftDesiredPosition;
  // private double rightDesiredPosition;

  private HandStates handState;
  private HandStates desiredState;

  private int handLeftZeroStableCounts;
  // private int handRightZeroStableCounts;
  private int hasPieceStableCounts;
  private int closingStableCounts;

  private boolean leftZeroDone;
  private static Constants constants;
  private double lastPercent = 0.0;
  // private boolean rightZeroDone;

  public HandSubsystem(Constants constants) {
    HandSubsystem.constants = constants;
    handLeftTalon = new TalonSRX(Constants.HandConstants.kHandTalonId);
    handLeftTalon.configFactoryDefault();
    handLeftTalon.configAllSettings(Constants.HandConstants.getHandTalonConfig());
    handLeftTalon.configSupplyCurrentLimit(Constants.HandConstants.getHandSupplyLimitConfig());
    handLeftTalon.setNeutralMode(NeutralMode.Brake);

    rollerTalon = new TalonSRX(HandConstants.kRollerTalonId);
    rollerTalon.configFactoryDefault();
    rollerTalon.configAllSettings(HandConstants.getRolleConfig());
    rollerTalon.configSupplyCurrentLimit(HandConstants.getRollerSupplyLimitConfig());
    rollerTalon.configForwardLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    rollerTalon.configReverseLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    rollerTalon.setNeutralMode(NeutralMode.Coast);

    // handRightTalon = new TalonSRX(Constants.HandConstants.kWristTalonId);
    // handRightTalon.configAllSettings(Constants.HandConstants.getHandTalonConfig());
    // handRightTalon.configSupplyCurrentLimit(Constants.HandConstants.getHandSupplyLimitConfig());

    leftDesiredPosition = getLeftPos();
    // rightDesiredPosition = getRightPos();

    handState = HandStates.IDLE;
    handLeftZeroStableCounts = 0;
    closingStableCounts = 0;
    zeroHand();
    // handRightZeroStableCounts = 0;
  }

  public void runRollers(double percent) {
    if (percent != lastPercent) logger.info("Running rollers at {}", percent);
    rollerTalon.set(TalonSRXControlMode.PercentOutput, percent);
    lastPercent = percent;
  }

  public double getRollersVel() {
    return rollerTalon.getSelectedSensorVelocity();
  }

  public void setLeftPos(double location) {
    if (leftDesiredPosition != location) logger.info("Hand (left) moving to {}", location);
    leftDesiredPosition = location;
    handLeftTalon.set(ControlMode.MotionMagic, location);
  }

  // public void setRightPos(double location) {
  //   logger.info("Hand (right) moving to {}", location);
  //   rightDesiredPosition = location;
  //   handRightTalon.set(ControlMode.MotionMagic, location);
  // }

  private void setPos(double leftLocation /*, double rightLocation*/) {
    setLeftPos(leftLocation);
    // setRightPos(rightLocation);
  }

  public void setLeftPct(double pct) {
    logger.info("Hand (left) speed: {}", pct);
    handLeftTalon.set(ControlMode.PercentOutput, pct);
  }

  public void toggleHandPos() {}

  // public void setRightPct(double pct) {
  //   logger.info("Hand (right) speed: {}", pct);
  //   handRightTalon.set(ControlMode.PercentOutput, pct);
  // }

  public double getLeftPos() {
    return handLeftTalon.getSelectedSensorPosition();
  }

  public void stowHand(double leftLocation) {
    logger.info("{} -> STOW_CLOSED", handState);
    handState = HandStates.STOW_CLOSED;
    setPos(leftLocation);
  }

  // public double getRightPos() {
  //   return handRightTalon.getSelectedSensorPosition();
  // }

  public boolean isFinished() {
    if (handState == HandStates.TRANSITIONING && desiredState == HandStates.CONE_CLOSED) {
      if (closingStableCounts >= HandConstants.kHoldingStableCounts) {
        return true;
      }
    } else if (handState == HandStates.CONE_CLOSED) return true;

    return Math.abs(leftDesiredPosition - getLeftPos())
        <= Constants.HandConstants
            .kAllowedError /*&& Math.abs(rightDesiredPosition - getRightPos()) <= Constants.HandConstants.kAllowedError*/;
  }

  public void zeroHand() {
    // leftZeroDone = false;
    // // rightZeroDone = false;

    // handLeftTalon.configForwardSoftLimitEnable(false);
    // handLeftTalon.configReverseSoftLimitEnable(false);

    // // handRightTalon.configForwardSoftLimitEnable(false);
    // // handRightTalon.configReverseSoftLimitEnable(false);

    // handLeftTalon.configSupplyCurrentLimit(
    //     Constants.HandConstants.getHandZeroSupplyCurrentLimit(), Constants.kTalonConfigTimeout);
    // // handRightTalon.configSupplyCurrentLimit(
    // //     Constants.HandConstants.getHandZeroSupplyCurrentLimit(),
    // Constants.kTalonConfigTimeout);

    // setLeftPct(Constants.HandConstants.kHandZeroSpeed);
    // // setRightPct(Constants.HandConstants.kHandZeroSpeed);

    double absolute = handLeftTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    double offset = absolute - constants.kHandZeroTicks;
    handLeftTalon.setSelectedSensorPosition(offset);

    logger.info(
        "Absolute: {}, Zero pos: {}, Offset: {}", absolute, constants.kHandZeroTicks, offset);

    // logger.info("Hand is zeroing");
    // handState = HandStates.ZEROING;
  }

  public void open() {
    logger.info("Opening hand");
    setPos(Constants.HandConstants.kHandOpenPosition);
    desiredState = HandStates.OPEN;
    handState = HandStates.TRANSITIONING;
  }

  public void openIntake() {
    logger.info("Opening Hand to Intake Position");
    setPos(Constants.HandConstants.kIntakeOpenPosition);
    desiredState = HandStates.OPEN;
    handState = HandStates.TRANSITIONING;
  }

  public void openFloor() {
    logger.info("Opening hand to Floor Position");
    setPos(Constants.HandConstants.kFloorOpenPosition);
    desiredState = HandStates.OPEN;
    handState = HandStates.TRANSITIONING;
  }

  public void openShelf() {
    logger.info("Opening Hand to Shelf Position");
    setPos(Constants.HandConstants.kShelfOpenPosition);
    desiredState = HandStates.OPEN;
    handState = HandStates.TRANSITIONING;
  }

  public double getSensor() {
    return handLeftTalon.getSensorCollection().getAnalogInRaw();
  }

  public boolean hasCone() {
    if (rollerTalon.isFwdLimitSwitchClosed() > 0) {
      hasPieceStableCounts++;
    } else hasPieceStableCounts = 0;

    return hasPieceStableCounts > Constants.HandConstants.kHasConeStableCounts;
  }

  public boolean hasCube() {
    if (rollerTalon.isRevLimitSwitchClosed() > 0) {
      hasPieceStableCounts++;
    } else hasPieceStableCounts = 0;

    return hasPieceStableCounts > Constants.HandConstants.kHasCubeStableCounts;
  }

  public void grabCube() {
    if (!((desiredState == HandStates.CUBE_CLOSED && handState == HandStates.TRANSITIONING)
        || (handState == HandStates.CUBE_CLOSED))) {
      logger.info("Grabbing cube");
      // runRollers(HandConstants.kRollerOutCube);
      setPos(Constants.HandConstants.kCubeGrabbingPosition /*,
          Constants.HandConstants.kCubeGrabbingPositionRight*/);
      desiredState = HandStates.CUBE_CLOSED;
      handState = HandStates.TRANSITIONING;
    }
  }

  public void grabCone() {
    if (!((desiredState == HandStates.CONE_CLOSED && handState == HandStates.TRANSITIONING)
        || (handState == HandStates.CONE_CLOSED))) {
      logger.info("Grabbing cone");
      setPos(Constants.HandConstants.kConeGrabbingPosition /*,
          Constants.HandConstants.kConeGrabbingPositionRight*/);
      desiredState = HandStates.CONE_CLOSED;
      handState = HandStates.TRANSITIONING;
      closingStableCounts = 0;
      hasPieceStableCounts = 0;
    }
  }

  public HandStates getHandState() {
    return handState;
  }

  public HandStates getDesiredHandState() {
    return desiredState;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(handLeftTalon);
    telemetryService.register(rollerTalon);
    // telemetryService.register(handRightTalon);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Hand State", () -> getHandState().ordinal()));
  }

  @Override
  public void periodic() {
    switch (handState) {
      case IDLE:
        break;
      case ZEROING:
        if (Math.abs(handLeftTalon.getSelectedSensorVelocity())
            < Constants.HandConstants.kZeroTargetSpeedTicksPer100ms) {
          handLeftZeroStableCounts++;
        } else {
          handLeftZeroStableCounts = 0;
        }

        // if (Math.abs(handRightTalon.getSelectedSensorVelocity())
        //     < Constants.HandConstants.kZeroTargetSpeedTicksPer100ms) {
        //   handRightZeroStableCounts++;
        // } else {
        //   handRightZeroStableCounts = 0;
        // }

        if (!leftZeroDone && handLeftZeroStableCounts > Constants.HandConstants.kZeroStableCounts) {
          handLeftTalon.setSelectedSensorPosition(0.0);
          handLeftTalon.configSupplyCurrentLimit(
              Constants.HandConstants.getHandSupplyLimitConfig(), Constants.kTalonConfigTimeout);

          handLeftTalon.configForwardSoftLimitEnable(true);
          handLeftTalon.configReverseSoftLimitEnable(true);

          setLeftPct(0);
          // leftDesiredPosition = 0;
          leftZeroDone = true;
        }

        // if (!rightZeroDone
        //     && handRightZeroStableCounts > Constants.HandConstants.kZeroStableCounts) {
        //   handRightTalon.setSelectedSensorPosition(0.0);
        //   handRightTalon.configSupplyCurrentLimit(
        //       Constants.HandConstants.getHandSupplyLimitConfig(), Constants.kTalonConfigTimeout);

        //   handRightTalon.configForwardSoftLimitEnable(true);
        //   handRightTalon.configReverseSoftLimitEnable(true);

        //   setRightPct(0);
        //   rightDesiredPosition = 0;
        //   rightZeroDone = true;
        // }

        if (handLeftZeroStableCounts > Constants.HandConstants.kZeroStableCounts
        /*&& handRightZeroStableCounts > Constants.HandConstants.kZeroStableCounts*/ ) {
          handState = HandStates.ZEROED;
        }

        break;
      case ZEROED:
        // logger.info("Hand is zeroed");
        break;
      default:
        break;

      case OPEN:
        // logger.info("Hand is open");
        break;

      case CUBE_CLOSED:
        // logger.info("Hand is closed for cube");
        break;

      case CONE_CLOSED:
        // logger.info("Hand is closed for cone");
        break;
      case STOW_CLOSED:
        break;
      case TRANSITIONING:
        if (desiredState == HandStates.CONE_CLOSED) {
          if (Math.abs(handLeftTalon.getSelectedSensorVelocity())
                  < Constants.HandConstants.kHoldingVelocityThreshold
              && getLeftPos() >= HandConstants.kHoldingTickThreshold
              && Math.abs(getRollersVel()) <= HandConstants.kConeVelLimit
              && hasCone()) {
            closingStableCounts++;
          } else {
            closingStableCounts = 0;
          }

          if (isFinished()) {
            runRollers(HandConstants.kRollerConeHoldSpeed);
            handState = desiredState;
          }
        }
        if (isFinished()) {
          handState = desiredState;
        }
        break;
    }
  }

  public enum HandStates {
    IDLE,
    ZEROING,
    ZEROED,
    OPEN,
    CUBE_CLOSED,
    CONE_CLOSED,
    STOW_CLOSED,
    TRANSITIONING
  }
}
