package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ArmSubsystem extends MeasurableSubsystem {
  private Logger logger = LoggerFactory.getLogger(ArmSubsystem.class);

  private ShoulderSubsystem shoulderSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ElbowSubsystem elbowSubsystem;
  private ArmState desiredState;
  private ArmState armState;

  public ArmSubsystem(
      ShoulderSubsystem shoulderSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElbowSubsystem elbowSubsystem) {
    this.armState = ArmState.STOW; // Maybe not?
    this.desiredState = ArmState.STOW;
    this.shoulderSubsystem = shoulderSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elbowSubsystem = elbowSubsystem;
  }

  public void toStowPos() {
    desiredState = ArmState.STOW;
  }

  public void toIntakePos() {
    desiredState = ArmState.INTAKE;
  }

  public void toLowPos() {
    desiredState = ArmState.LOW;
  }

  public void toMidPos() {
    desiredState = ArmState.MID;
  }

  public void toHighPos() {
    desiredState = ArmState.HIGH;
  }

  public void toShelfPos() {
    desiredState = ArmState.SHELF;
  }

  public boolean isFinished() {
    return shoulderSubsystem.isFinished()
        && elevatorSubsystem.isFinished()
        && elbowSubsystem.isFinished();
  }

  private void attemptSetArmState(ArmState newArmState) {
    if (isFinished()) {
      shoulderSubsystem.setPos(armState.shoulderPos);
      elevatorSubsystem.setPos(armState.elevatorPos);
      elbowSubsystem.rotateClosedLoop((int) armState.elbowPos);

      this.armState = newArmState;

      logger.info("Setting armState to {}", newArmState.name());
    } else {
      logger.info("Arm not finished moving, cannot proceed to next position.");
    }
  }

  public HandRegion getHandRegion() {
    Translation2d handPos = getHandPosition();

    if (handPos.getX() >= ArmConstants.kFrontBumperX && handPos.getY() < ArmConstants.kCamY) {
      return HandRegion.BUMPER;
    } else if ((handPos.getX() >= Constants.ArmConstants.kHouseMinX
            && handPos.getX() <= Constants.ArmConstants.kFrontBumperX)
        && (handPos.getY()
            <= Constants.ArmConstants.kHouseLineSlope * handPos.getX()
                + Constants.ArmConstants.kHouseIntercept)) {
      return HandRegion.HOUSE;
    } else if (handPos.getX() <= ArmConstants.kIntakeX) {
      if (handPos.getY() <= ArmConstants.kIntakeMaxY) {
        return HandRegion.INSIDE_INTAKE;
      } else {
        return HandRegion.ABOVE_INTAKE;
      }
    } else if (handPos.getX() < ArmConstants.kHouseMinX && handPos.getX() > ArmConstants.kIntakeX) {
      if (shoulderSubsystem.getPos() <= Constants.ArmConstants.kShoulderVerticalMax) {
        return HandRegion.HOUSE;
      } else {
        return HandRegion.INTAKE;
      }
    }

    // } else if (handPos.getX() <= Constants.ArmConstants.kHouseMinX
    // && handPos.getY() <= ArmConstants.kIntakeMaxY) {
    // if (handPos.getX() <= Constants.ArmConstants.kIntakeX) {
    // return HandRegion.INSIDE_INTAKE;
    // } else {
    // return HandRegion.INTAKE;
    // }
    // } else if (handPos.getY() >= ArmConstants.kIntakeMaxY
    // && handPos.getX() <= Constants.ArmConstants.kHouseMinX) {
    // if (shoulderSubsystem.getPos() <=
    // Constants.ArmConstants.kShoulderVerticalMax) {
    // // return HandRegion.FRONT;
    // return HandRegion.ABOVE_INTAKE;
    // } else {
    // return HandRegion.INTAKE;
    // }
    // }

    return HandRegion.FRONT; // Assume arm is free
  }

  public Translation2d getHandPosition() {
    final double f = Constants.ShoulderConstants.kElevatorBaseToPivot;

    double elbowAngleWithGround =
        Math.toRadians(shoulderSubsystem.getDegs() - (90.0 - elbowSubsystem.getRelativeDegs()));
    double shoulderAngle = Math.toRadians(shoulderSubsystem.getDegs());

    double x =
        (elevatorSubsystem.getExtensionMeters()) * Math.cos(shoulderAngle)
            - f * Math.cos(Math.PI / 2 - shoulderAngle)
            + Constants.ElbowConstants.kLength * Math.cos(elbowAngleWithGround)
            + ArmConstants.kelevatorToElbowPivot * Math.cos(Math.PI / 2 - shoulderAngle);
    double y =
        (elevatorSubsystem.getExtensionMeters() + f / Math.tan(shoulderAngle))
                * Math.sin(shoulderAngle)
            + Constants.ElbowConstants.kLength * Math.sin(elbowAngleWithGround)
            - ArmConstants.kelevatorToElbowPivot * Math.sin(Math.PI / 2 - shoulderAngle);

    return new Translation2d(x, y);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Hand X", () -> getHandPosition().getX()),
        new Measure("Hand Y", () -> getHandPosition().getY()),
        new Measure("Hand region", () -> getHandRegion().ordinal()));
  }

  @Override
  public void periodic() {
    HandRegion currHandRegion = getHandRegion();

    shoulderSubsystem.setSoftLimits(
        currHandRegion.minTicksShoulder, currHandRegion.maxTicksShoulder);
    elevatorSubsystem.setSoftLimits(
        currHandRegion.minTicksElevator, currHandRegion.maxTicksElevator);
    elbowSubsystem.setSoftLimits(currHandRegion.minTicksElbow, currHandRegion.maxTicksElbow);

    switch (armState) {
      case STOW:
        if (desiredState == ArmState.LOW) {
          attemptSetArmState(ArmState.LOW_ELBOW);
        } else if (desiredState == ArmState.MID) {
          attemptSetArmState(ArmState.MID_ELBOW);
        } else if (desiredState == ArmState.HIGH) {
          attemptSetArmState(ArmState.HIGH_ELBOW);
        } else if (desiredState == ArmState.INTAKE) {
          attemptSetArmState(ArmState.INTAKE_ELEVATOR);
        } else if (desiredState == ArmState.SHELF) {
          attemptSetArmState(ArmState.SHELF);
        }
        break;
      case INTAKE:
        if (desiredState != ArmState.INTAKE) {
          attemptSetArmState(ArmState.TRANSITION_SHOULDER);
        }
        break;
      case LOW:
        if (desiredState != ArmState.LOW) {
          attemptSetArmState(ArmState.LOW_SHOULDER);
        }
        break;
      case MID:
        if (desiredState != ArmState.MID) {
          attemptSetArmState(ArmState.MID_ELBOW);
        }
        break;
      case HIGH:
        if (desiredState != ArmState.HIGH) {
          attemptSetArmState(ArmState.HIGH_ELBOW);
        }
        break;
      case SHELF:
        if (desiredState != ArmState.SHELF) {
          attemptSetArmState(ArmState.STOW);
        }
        break;
      case MANUAL:
        break;

      // Transitional states
      case LOW_ELBOW:
        if (desiredState == ArmState.LOW) {
          attemptSetArmState(ArmState.LOW_SHOULDER);
        } else {
          attemptSetArmState(ArmState.STOW);
        }
        break;
      case LOW_SHOULDER:
        if (desiredState == ArmState.LOW) {
          attemptSetArmState(ArmState.LOW);
        } else {
          attemptSetArmState(ArmState.LOW_ELBOW);
        }
        break;
      case MID_ELBOW:
        if (desiredState == ArmState.MID) {
          attemptSetArmState(ArmState.MID);
        } else {
          attemptSetArmState(ArmState.STOW);
        }
        break;
      case HIGH_ELBOW:
        if (desiredState == ArmState.HIGH) {
          attemptSetArmState(ArmState.HIGH);
        } else {
          attemptSetArmState(ArmState.STOW);
        }
        break;
      case INTAKE_ELEVATOR:
        attemptSetArmState(ArmState.INTAKE);
        break;
      case TRANSITION_SHOULDER:
        attemptSetArmState(ArmState.STOW_ELEVATOR);
        break;
      case STOW_ELEVATOR:
        attemptSetArmState(ArmState.STOW_SHOULDER);
        break;
      case STOW_SHOULDER:
        attemptSetArmState(ArmState.STOW);
        break;
      default:
        break;
    }
  }

  public enum ArmState {
    STOW(0, 0, 0),
    INTAKE(0, 0, 0),
    LOW(0, 0, 0),
    MID(0, 0, 0),
    HIGH(0, 0, 0),
    SHELF(0, 0, 0),
    MANUAL(0, 0, 0),
    LOW_ELBOW(0, 0, 0),
    LOW_SHOULDER(0, 0, 0),
    MID_ELBOW(0, 0, 0),
    HIGH_ELBOW(0, 0, 0),
    INTAKE_ELEVATOR(0, 0, 0),
    TRANSITION_SHOULDER(0, 0, 0),
    STOW_ELEVATOR(0, 0, 0),
    STOW_SHOULDER(0, 0, 0);

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
        ArmConstants.kShoulderPhysicalMin,
        ArmConstants.kShoulderPhysicalMax,
        ArmConstants.kElevatorBumperMin,
        ArmConstants.kElevatorBumperMax,
        ArmConstants.kElbowBumperMin,
        ArmConstants.kElbowBumperMax),
    FRONT(
        ArmConstants.kShoulderPhysicalMin,
        ArmConstants.kShoulderPhysicalMax,
        ArmConstants.kElevatorPhysicalMin,
        ArmConstants.kElevatorPhysicalMax,
        ArmConstants.kElbowPhysicalMin,
        ArmConstants.kElbowPhysicalMax),
    HOUSE(
        ArmConstants.kShoulderVerticalMin,
        ArmConstants.kShoulderVerticalMax,
        ArmConstants.kElevatorHouseMin,
        ArmConstants.kElevatorHouseMax,
        ArmConstants.kElbowPhysicalMin,
        ArmConstants.kElbowPhysicalMax),
    INTAKE(
        ArmConstants.kShoulderPhysicalMin,
        ArmConstants.kShoulderPhysicalMax,
        ArmConstants.kElevatorPhysicalMin,
        ArmConstants.kElevatorPhysicalMax,
        ArmConstants.kElbowIntakeMin,
        ArmConstants.kElbowIntakeMax),
    ABOVE_INTAKE(
        ArmConstants.kShoulderPhysicalMin,
        ArmConstants.kShoulderPhysicalMax,
        ArmConstants.kElevatorPhysicalMin,
        ArmConstants.kElevatorPhysicalMax,
        ArmConstants.kElbowAboveIntakeMin,
        ArmConstants.kElbowPhysicalMax),
    INSIDE_INTAKE(
        ArmConstants.kShoulderPhysicalMin,
        ArmConstants.kShoulderPhysicalMax,
        ArmConstants.kElevatorPhysicalMin,
        ArmConstants.kElevatorBelowIntakeMax,
        ArmConstants.kElbowInsideIntakeMin,
        ArmConstants.kElbowIntakeMax),

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
