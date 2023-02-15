package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShoulderConstants;
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
  private ArmState currState;
  private CurrentAxis currAxis;

  public ArmSubsystem(
      ShoulderSubsystem shoulderSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElbowSubsystem elbowSubsystem) {
    this.currState = ArmState.STOW; // Maybe not?
    this.desiredState = ArmState.STOW;
    this.shoulderSubsystem = shoulderSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elbowSubsystem = elbowSubsystem;
  }

  public void toStowPos() {
    toStowPos(ArmState.STOW);
  }

  public void toStowPos(ArmState desiredState) {
    switch (currState) {
      case LOW: // Fall through
      case MID:
      case HIGH:
        logger.info("{} -> SCORE_TO_STOW", currState);
        currState = ArmState.SCORE_TO_STOW;
        currAxis = CurrentAxis.SHOULDER;
        shoulderSubsystem.setPos(ArmState.STOW.shoulderPos);
        break;
      case INTAKE_STAGE: // Fall though
      case INTAKE:
        logger.info("{} -> INTAKE_TO_STOW", currState);
        currState = ArmState.INTAKE_TO_STOW;
        currAxis = CurrentAxis.ELEVATOR;
        shoulderSubsystem.setPos(ArmState.STOW.shoulderPos);
        elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
        break;
      case SHELF:
        logger.info("{} -> SHELF_TO_STOW", currState);
        currState = ArmState.SHELF_TO_STOW;
        currAxis = CurrentAxis.ELEVATOR;
        shoulderSubsystem.setPos(ArmState.STOW.shoulderPos);
        elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
      case FLOOR:
        logger.info("{} -> FLOOR_TO_STOW", currState);
        currState = ArmState.FLOOR_TO_STOW;
        currAxis = CurrentAxis.SHOULDER;
        shoulderSubsystem.setPos(ArmState.STOW.shoulderPos);
        break;
      default:
        logger.info("{} -> SCORE_TO_STOW", currState);
        currState = ArmState.SCORE_TO_STOW;
        currAxis = CurrentAxis.SHOULDER;
        shoulderSubsystem.setPos(ArmState.STOW.shoulderPos);
        break;
    }
    this.desiredState = desiredState;
  }

  public void toIntakeStagePos() {
    switch (currState) {
      case STOW:
        logger.info("{} -> STOW_TO_INTAKE_STAGE", currState);
        currState = ArmState.STOW_TO_INTAKE_STAGE;
        currAxis = CurrentAxis.ELBOW;
        shoulderSubsystem.setPos(ArmState.INTAKE_STAGE.shoulderPos);
        elbowSubsystem.setPos(ArmState.INTAKE_STAGE.elbowPos);
        break;
      default:
        toStowPos(ArmState.INTAKE_STAGE);
        break;
    }
    desiredState = ArmState.INTAKE_STAGE;
  }

  public void toIntakePos() {
    switch (currState) {
      case INTAKE_STAGE:
        logger.info("{} -> INTAKE_STAGE_TO_INTAKE", currState);
        currState = ArmState.INTAKE_STAGE_TO_INTAKE;
        currAxis = CurrentAxis.ELEVATOR;
        elevatorSubsystem.setPos(ArmState.INTAKE.elevatorPos);
        break;
      default:
        toIntakeStagePos();
        break;
    }
  }

  public void toLowPos() {
    switch (currState) {
      case STOW:
        logger.info("{} -> STOW_TO_LOW", currState);
        currState = ArmState.STOW_TO_LOW;
        currAxis = CurrentAxis.ELBOW;
        elbowSubsystem.setPos(ArmState.LOW.elbowPos);
        break;
      default:
        toStowPos(ArmState.LOW);
        break;
    }
    desiredState = ArmState.LOW;
  }

  public void toMidPos() {
    switch (currState) {
      case STOW:
        logger.info("{} -> STOW_TO_MID", currState);
        currState = ArmState.STOW_TO_MID;
        currAxis = CurrentAxis.ELBOW;
        elbowSubsystem.setPos(ArmState.MID.elbowPos);
        break;
      default:
        toStowPos(ArmState.MID);
        break;
    }
    desiredState = ArmState.MID;
  }

  public void toHighPos() {
    switch (currState) {
      case STOW:
        logger.info("{} -> STOW_TO_HIGH", currState);
        currState = ArmState.STOW_TO_HIGH;
        currAxis = CurrentAxis.ELBOW;
        elbowSubsystem.setPos(ArmState.HIGH.elbowPos);
        break;
      default:
        toStowPos(ArmState.HIGH);
        break;
    }
    desiredState = ArmState.HIGH;
  }

  public void toShelfPos() {
    switch (currState) {
      case STOW:
        logger.info("{} -> STOW_TO_SHELF", currState);
        currState = ArmState.STOW_TO_SHELF;
        currAxis = CurrentAxis.ELBOW;
        elbowSubsystem.setPos(ArmState.SHELF.elbowPos);
        shoulderSubsystem.setPos(ArmState.SHELF.shoulderPos);
        break;
      default:
        toStowPos(ArmState.SHELF);
        break;
    }
    desiredState = ArmState.SHELF;
  }

  public void toFloorPos() {
    switch (currState) {
      case STOW:
        logger.info("{} -> STOW_TO_FLOOR", currState);
        currState = ArmState.STOW_TO_FLOOR;
        currAxis = CurrentAxis.ELBOW;
        elbowSubsystem.setPos(ArmState.FLOOR.elbowPos);
        break;
      default:
        toStowPos(ArmState.FLOOR);
        break;
    }
    desiredState = ArmState.FLOOR;
  }

  public boolean isFinished() {
    return shoulderSubsystem.isFinished()
        && elevatorSubsystem.isFinished()
        && elbowSubsystem.isFinished();
  }

  private void attemptSetArmState(ArmState newArmState) {
    if (isFinished()) {
      shoulderSubsystem.setPos(currState.shoulderPos);
      elevatorSubsystem.setPos(currState.elevatorPos);
      elbowSubsystem.setPos((int) currState.elbowPos);

      this.currState = newArmState;

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
    } else if (handPos.getX() < ArmConstants.kHouseMinX) {
      if (elevatorSubsystem.getPos() <= ArmConstants.kElevatorHouseMin) {
        return HandRegion.INSIDE_INTAKE;
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
            + ArmConstants.kElevatorToElbowPivot * Math.cos(Math.PI / 2 - shoulderAngle);
    double y =
        (elevatorSubsystem.getExtensionMeters() + f / Math.tan(shoulderAngle))
                * Math.sin(shoulderAngle)
            + Constants.ElbowConstants.kLength * Math.sin(elbowAngleWithGround)
            - ArmConstants.kElevatorToElbowPivot * Math.sin(Math.PI / 2 - shoulderAngle);

    return new Translation2d(x, y);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Hand X", () -> getHandPosition().getX()),
        new Measure("Hand Y", () -> getHandPosition().getY()),
        new Measure("Hand region", () -> getHandRegion().ordinal()));
  }

  public ArmState getCurrState() {
    return currState;
  }

  @Override
  public void periodic() {
    HandRegion currHandRegion = getHandRegion();

    shoulderSubsystem.setSoftLimits(
        currHandRegion.minTicksShoulder, currHandRegion.maxTicksShoulder);
    elevatorSubsystem.setSoftLimits(
        currHandRegion.minTicksElevator, currHandRegion.maxTicksElevator);
    elbowSubsystem.setSoftLimits(currHandRegion.minTicksElbow, currHandRegion.maxTicksElbow);

    switch (currState) {
      case STOW:
        switch (desiredState) {
          case LOW:
            toLowPos();
            break;
          case MID:
            toMidPos();
            break;
          case HIGH:
            toHighPos();
            break;
          case INTAKE_STAGE:
            toIntakePos();
            break;
          case SHELF:
            toShelfPos();
            break;
          case FLOOR:
            toFloorPos();
            break;
          default:
            break;
        }
        break;
      case INTAKE_STAGE:
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
      case FLOOR:
        break;
      case MANUAL:
        break;
      case STOW_TO_INTAKE_STAGE:
        if (elbowSubsystem.isFinished()) {
          logger.info("{} -> INTAKE_STAGE", currState);
          currState = ArmState.INTAKE_STAGE;
        }
        break;
      case STOW_TO_LOW:
        switch (currAxis) {
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              currAxis = CurrentAxis.SHOULDER;
              shoulderSubsystem.setPos(ArmState.LOW.shoulderPos);
            }
            break;
          case SHOULDER:
            if (shoulderSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELEVATOR;
              elevatorSubsystem.setPos(ArmState.LOW.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              logger.info("{} -> LOW", currState);
              currState = ArmState.LOW;
            }
            break;
          default:
            break;
        }
        break;
      case STOW_TO_MID:
        switch (currAxis) {
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELEVATOR;
              elevatorSubsystem.setPos(ArmState.MID.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              currAxis = CurrentAxis.SHOULDER;
              shoulderSubsystem.setPos(ArmState.MID.shoulderPos);
            }
            break;
          case SHOULDER:
            if (shoulderSubsystem.isFinished()) {
              logger.info("{} -> MID", currState);
              currState = ArmState.MID;
            }
            break;
          default:
            break;
        }
        break;
      case STOW_TO_HIGH:
        switch (currAxis) {
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELEVATOR;
              elevatorSubsystem.setPos(ArmState.HIGH.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              currAxis = CurrentAxis.SHOULDER;
              shoulderSubsystem.setPos(ArmState.HIGH.shoulderPos);
            }
            break;
          case SHOULDER:
            if (shoulderSubsystem.isFinished()) {
              logger.info("{} -> HIGH", currState);
              currState = ArmState.HIGH;
            }
            break;
          default:
            break;
        }
        break;
      case STOW_TO_SHELF:
        switch (currAxis) {
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELEVATOR;
              elevatorSubsystem.setPos(ArmState.SHELF.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              logger.info("{} -> SHELF", currState);
              currState = ArmState.SHELF;
            }
            break;
          default:
            break;
        }
        break;
      case STOW_TO_FLOOR:
        switch (currAxis) {
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              currAxis = CurrentAxis.SHOULDER;
              shoulderSubsystem.setPos(ArmState.FLOOR.shoulderPos);
            }
            break;
          case SHOULDER:
            if (shoulderSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELEVATOR;
              elevatorSubsystem.setPos(ArmState.FLOOR.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              logger.info("{} -> FLOOR", currState);
              currState = ArmState.FLOOR;
            }
            break;
          default:
            break;
        }
        break;
      case SCORE_TO_STOW:
        switch (currAxis) {
          case SHOULDER:
            if (shoulderSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELEVATOR;
              elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELBOW;
              elbowSubsystem.setPos(ArmState.STOW.elbowPos);
            }
            break;
          case ELBOW:
            if (elbowSubsystem.isFinished()) {

              logger.info("{} -> STOW", currState);
              currState = ArmState.STOW;
            }
            break;

          default:
            break;
        }
        break;
        // STOW TO INTAKE STAGE
        // STOW TO FLOOR
        // INTAKE STAGE TO INTAKE

        // Transitional states
        // case LOW_ELBOW:
        //   if (desiredState == ArmState.LOW) {
        //     attemptSetArmState(ArmState.LOW_SHOULDER);
        //   } else {
        //     attemptSetArmState(ArmState.STOW);
        //   }
        //   break;
        // case LOW_SHOULDER:
        //   if (desiredState == ArmState.LOW) {
        //     attemptSetArmState(ArmState.LOW);
        //   } else {
        //     attemptSetArmState(ArmState.LOW_ELBOW);
        //   }
        //   break;
        // case MID_ELBOW:
        //   if (desiredState == ArmState.MID) {
        //     attemptSetArmState(ArmState.MID);
        //   } else {
        //     attemptSetArmState(ArmState.STOW);
        //   }
        //   break;
        // case HIGH_ELBOW:
        //   if (desiredState == ArmState.HIGH) {
        //     attemptSetArmState(ArmState.HIGH);
        //   } else {
        //     attemptSetArmState(ArmState.STOW);
        //   }
        //   break;
        // case INTAKE_ELEVATOR:
        //   attemptSetArmState(ArmState.INTAKE);
        //   break;
        // case TRANSITION_SHOULDER:
        //   attemptSetArmState(ArmState.STOW_ELEVATOR);
        //   break;
        // case STOW_ELEVATOR:
        //   attemptSetArmState(ArmState.STOW_SHOULDER);
        //   break;
        // case STOW_SHOULDER:
        //   attemptSetArmState(ArmState.STOW);
        //   break;
        // default:
        //   break;
    }
  }

  public enum ArmState {
    STOW(
        ShoulderConstants.kStowShoulder,
        ElevatorConstants.kStowElevator,
        ElbowConstants.kStowElbow),
    INTAKE(
        ShoulderConstants.kIntakeShoulder,
        ElevatorConstants.kIntakeElevator,
        ElbowConstants.kIntakeElbow),
    INTAKE_STAGE(
        ShoulderConstants.kIntakeShoulder,
        ElevatorConstants.kStowElevator,
        ElbowConstants.kIntakeElbow),
    LOW(
        ShoulderConstants.kLevelOneShoulder,
        ElevatorConstants.kLevelOneElevator,
        ElbowConstants.kLevelOneElbow),
    MID(
        ShoulderConstants.kLevelTwoShoulder,
        ElevatorConstants.kLevelTwoElevator,
        ElbowConstants.kLevelTwoElbow),
    HIGH(
        ShoulderConstants.kLevelThreeShoulder,
        ElevatorConstants.kLevelThreeElevator,
        ElbowConstants.kLevelThreeElbow),
    SHELF(
        ShoulderConstants.kShelfShoulder,
        ElevatorConstants.kShelfElevator,
        ElbowConstants.kShelfElbow),
    FLOOR(
        ShoulderConstants.kFloorShoulder,
        ElevatorConstants.kFloorElevator,
        ElbowConstants.kFloorElbow),
    MANUAL(0, 0, 0),

    STOW_TO_INTAKE_STAGE(0, 0, 0),
    STOW_TO_LOW(0, 0, 0),
    STOW_TO_MID(0, 0, 0),
    STOW_TO_HIGH(0, 0, 0),
    STOW_TO_SHELF(0, 0, 0),
    STOW_TO_FLOOR(0, 0, 0),
    INTAKE_STAGE_TO_INTAKE(0, 0, 0),
    INTAKE_TO_STOW(0, 0, 0),
    SCORE_TO_STOW(0, 0, 0),
    SHELF_TO_STOW(0, 0, 0),
    FLOOR_TO_STOW(0, 0, 0),
    MANUAL_TO_STOW(0, 0, 0);

    // LOW_ELBOW(0, 0, 0),
    // LOW_SHOULDER(0, 0, 0),
    // MID_ELBOW(0, 0, 0),
    // HIGH_ELBOW(0, 0, 0),
    // INTAKE_ELEVATOR(0, 0, 0),
    // TRANSITION_SHOULDER(0, 0, 0),
    // STOW_ELEVATOR(0, 0, 0),
    // STOW_SHOULDER(0, 0, 0);

    public final double shoulderPos;
    public final double elevatorPos;
    public final double elbowPos;

    ArmState(double shoulderPos, double elevatorPos, double elbowPos) {
      this.shoulderPos = shoulderPos;
      this.elevatorPos = elevatorPos;
      this.elbowPos = elbowPos;
    }
  }

  public enum CurrentAxis {
    ELBOW,
    ELEVATOR,
    SHOULDER,
    NONE;

    CurrentAxis() {}
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
        ArmConstants.kInsideIntakeElevatorMin,
        ArmConstants.kInsideIntakeElevatorMax,
        ArmConstants.kElbowInsideIntakeMin,
        ArmConstants.kElbowInsideIntakeMax),

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
