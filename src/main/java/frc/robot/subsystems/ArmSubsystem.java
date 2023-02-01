package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ArmSubsystem extends MeasurableSubsystem {
  private Logger logger = LoggerFactory.getLogger(ArmSubsystem.class);
  private ArmState armState;
  private ShoulderSubsystem shoulderSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ElbowSubsystem elbowSubsystem;

  public ArmSubsystem(
      ShoulderSubsystem shoulderSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElbowSubsystem elbowSubsystem) {
    this.armState = ArmState.STOWED; // Maybe not?
    this.shoulderSubsystem = shoulderSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elbowSubsystem = elbowSubsystem;
  }

  public void toStowPos() {
    setArmState(ArmState.STOWED);
  }

  public void toIntakePos() {
    setArmState(ArmState.INTAKE);
  }

  public void toLowPos() {
    setArmState(ArmState.LOW);
  }

  public void toMidPos() {
    setArmState(ArmState.MID);
  }

  public void toHighPos() {
    setArmState(ArmState.HIGH);
  }

  public void toShelfPos() {
    setArmState(ArmState.SHELF);
  }

  public boolean isFinished() {
    return shoulderSubsystem.isFinished() && elevatorSubsystem.isFinished()
    /*&& elbowSubsystem.isFinished()*/ ;
  }

  private void setArmState(ArmState newArmState) {
    this.armState = newArmState;

    shoulderSubsystem.setPos(armState.shoulderPos);
    elevatorSubsystem.setPos(armState.elevatorPos);
    elbowSubsystem.rotateClosedLoop((int) armState.elbowPos);

    logger.info("Setting armState to {}", newArmState.name());
  }

  public HandRegion getHandRegion() {
    Translation2d handPos = getHandPosition();

    if (handPos.getX() >= 0.45 && handPos.getY() < 0.16) {
      return HandRegion.BUMPER;
    } else if ((handPos.getX() >= 0 && handPos.getX() <= Constants.ArmConstants.kFrontBumperX)
        && (handPos.getY()
            <= Constants.ArmConstants.kHouseLineSlope * handPos.getX()
                + Constants.ArmConstants.kCamY)) {
      return HandRegion.HOUSE;
    }

    return HandRegion.FRONT; // Assume arm is free
  }

  public Translation2d getHandPosition() {
    final double f = Constants.ShoulderConstants.kElevatorBaseToPivot;

    double elbowAngleWithGround =
        Math.toRadians(shoulderSubsystem.getDegs() - (90.0 - elbowSubsystem.getRelativeDegs()));
    double shoulderAngle = Math.toRadians(shoulderSubsystem.getDegs());

    double x =
        elevatorSubsystem.getExtensionMeters() * Math.cos(shoulderAngle)
            - f / Math.sin(shoulderAngle)
            + Constants.ElbowConstants.kLength * Math.cos(elbowAngleWithGround);
    double y =
        (elevatorSubsystem.getExtensionMeters() + f / Math.tan(shoulderAngle))
                * Math.sin(shoulderAngle)
            + Constants.ElbowConstants.kLength * Math.sin(elbowAngleWithGround);

    return new Translation2d(x, y);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Hand X", () -> getHandPosition().getX()),
        new Measure("Hand Y", () -> getHandPosition().getY()));
  }

  @Override
  public void periodic() {
    switch (armState) {
      case STOWED:
        break;
      case INTAKE:
        break;
      case LOW:
        break;
      case MID:
        break;
      case HIGH:
        break;
      case SHELF:
        break;
      case OPENLOOP:
        break;
      default:
        break;
    }
  }

  public enum ArmState {
    STOWED(0, 0, 0),
    INTAKE(0, 0, 0),
    LOW(0, 0, 0),
    MID(0, 0, 0),
    HIGH(0, 0, 0),
    SHELF(0, 0, 0),
    OPENLOOP(0, 0, 0);

    public final double shoulderPos;
    public final double elevatorPos;
    public final double elbowPos;

    ArmState(double shoulderPos, double elevatorPos, double elbowPos) {
      this.shoulderPos = shoulderPos;
      this.elevatorPos = elevatorPos;
      this.elbowPos = elbowPos;
    }
  }

  public enum HandRegion {
    BUMPER(
        0,
        Constants.ShoulderConstants.kMaxFwd,
        0,
        Constants.ElevatorConstants.kMaxFwd,
        0,
        Constants.ElbowConstants.kForwardSoftLimit),
    FRONT(0, 0, 0, 0, 0, 0),
    HOUSE(0, 0, 0, 0, 0, 0),
    INTAKE(0, 0, 0, 0, 0, 0),
    UNKNOWN(0, 0, 0, 0, 0, 0);
    public final double minTicksShoulder,
        maxTicksShoulder,
        minTicksElevator,
        maxTicksElevator,
        minTicksElbow,
        maxTicksElbow;

    HandRegion(
        double minTicksShoulder,
        double maxTicksShoulder,
        double minTicksElevator,
        double maxTicksElevator,
        double minTicksElbow,
        double maxTicksElbow) {
      this.minTicksShoulder = minTicksShoulder;
      this.maxTicksShoulder = maxTicksShoulder;
      this.minTicksElevator = minTicksElevator;
      this.maxTicksElevator = maxTicksElevator;
      this.minTicksElbow = minTicksElbow;
      this.maxTicksElbow = maxTicksElbow;
    }
  }
}
