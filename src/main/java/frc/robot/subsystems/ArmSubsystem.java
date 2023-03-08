package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
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
  private boolean continueToIntake;
  private boolean continueToFloorSweep;
  private boolean isShoulderStaged = false;
  private boolean shouldFastStowArm = false;
  private boolean isArmFastStowing = false;
  private boolean hasElbowZeroed = true;
  private boolean isElbowReinforced = true;
  private boolean doReinforceElevator = true;

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

  public void setReinforceElevator(boolean doReinforceElevator) {
    this.doReinforceElevator = doReinforceElevator;
    logger.info("Turn Elevator Reinforce: {}", doReinforceElevator);
  }

  public void toStowPos() {
    toStowPos(ArmState.STOW);
  }

  public void toStowPos(ArmState desiredState) {
    switch (currState) {
      case LOW: // Fall through
      case MID_CONE:
      case MID_CUBE:
      case HIGH_CONE:
      case HIGH_CUBE:
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
    toIntakeStagePos(false);
  }

  public void toIntakeStagePos(boolean continueToIntake) {
    this.continueToIntake = continueToIntake;
    switch (currState) {
      case STOW:
        if (isElbowReinforced) {
          hasElbowZeroed = false;
          isElbowReinforced = false;
          elevatorSubsystem.unReinforceElevator();
          elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
        }
        if (!isElbowReinforced && elevatorSubsystem.isFinished()) {
          logger.info("{} -> STOW_TO_INTAKE_STAGE", currState);
          currAxis = CurrentAxis.ELBOW;
          shoulderSubsystem.setPos(ArmState.INTAKE_STAGE.shoulderPos);
          elbowSubsystem.setPos(ArmState.INTAKE_STAGE.elbowPos);
          currState = ArmState.STOW_TO_INTAKE_STAGE;
        }
        break;
      default:
        if (continueToIntake) {
          toStowPos(ArmState.INTAKE);
        } else {
          toStowPos(ArmState.INTAKE_STAGE);
        }
        break;
    }
    desiredState = ArmState.INTAKE_STAGE;
  }

  public void toIntakePos() {
    if (isElbowReinforced) {
      hasElbowZeroed = false;
      isElbowReinforced = false;
      elevatorSubsystem.unReinforceElevator();
      elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
    }
    if (!isElbowReinforced && elevatorSubsystem.isFinished()) {
      switch (currState) {
        case INTAKE_STAGE:
          logger.info("{} -> INTAKE_STAGE_TO_INTAKE", currState);
          currState = ArmState.INTAKE_STAGE_TO_INTAKE;
          currAxis = CurrentAxis.ELEVATOR;
          elevatorSubsystem.setPos(ArmState.INTAKE.elevatorPos);
          break;
        default:
          toIntakeStagePos(true);
          break;
      }
    }
  }

  public void toLowPos() {
    hasElbowZeroed = false;
    switch (currState) {
      case STOW:
        if (isElbowReinforced) {
          hasElbowZeroed = false;
          isElbowReinforced = false;
          elevatorSubsystem.unReinforceElevator();
          elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
        }
        if (!isElbowReinforced && elevatorSubsystem.isFinished()) {
          logger.info("{} -> STOW_TO_LOW", currState);
          currState = ArmState.STOW_TO_LOW;
          currAxis = CurrentAxis.ELBOW;
          elbowSubsystem.setPos(ArmState.LOW.elbowPos);
        }
        break;
      default:
        toStowPos(ArmState.LOW);
        break;
    }
    desiredState = ArmState.LOW;
  }

  public void toMidPos(GamePiece currGamePiece) {
    hasElbowZeroed = false;
    if (currGamePiece == GamePiece.NONE) {
      logger.info("Game piece is unknown yet required (toMidPos())! Defaulting to CONE");
    }

    desiredState = (currGamePiece == GamePiece.CUBE) ? ArmState.MID_CUBE : ArmState.MID_CONE;

    switch (currState) {
      case STOW:
        if (isElbowReinforced) {
          hasElbowZeroed = false;
          isElbowReinforced = false;
          elevatorSubsystem.unReinforceElevator();
          elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
        }
        if (!isElbowReinforced && elevatorSubsystem.isFinished()) {
          logger.info("{} -> STOW_TO_MID", currState);
          currState = ArmState.STOW_TO_MID;
          currAxis = CurrentAxis.ELBOW;
          elbowSubsystem.setPos(desiredState.elbowPos);
        }
        break;
      default:
        toStowPos(desiredState);
        break;
    }
  }

  public void toHighPos(GamePiece currGamePiece) {
    hasElbowZeroed = false;
    if (currGamePiece == GamePiece.NONE) {
      logger.info("Game piece is unknown yet required (toHighPos())! Defaulting to CONE");
    }

    desiredState = currGamePiece == GamePiece.CUBE ? ArmState.HIGH_CUBE : ArmState.HIGH_CONE;

    isShoulderStaged = false;
    switch (currState) {
      case STOW:
        if (isElbowReinforced) {
          hasElbowZeroed = false;
          isElbowReinforced = false;
          elevatorSubsystem.unReinforceElevator();
          elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
        }
        if (!isElbowReinforced && elevatorSubsystem.isFinished()) {
          logger.info("{} -> STOW_TO_HIGH", currState);
          currState = ArmState.STOW_TO_HIGH;
          currAxis = CurrentAxis.ELBOW;
          elbowSubsystem.setPos(desiredState.elbowPos);
        }
        break;
      default:
        toStowPos(desiredState);
        break;
    }
  }

  public void toShelfPos() {
    hasElbowZeroed = false;
    switch (currState) {
      case STOW:
        if (isElbowReinforced) {
          hasElbowZeroed = false;
          isElbowReinforced = false;
          elevatorSubsystem.unReinforceElevator();
          elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
        }
        if (!isElbowReinforced && elevatorSubsystem.isFinished()) {
          logger.info("{} -> STOW_TO_SHELF", currState);
          currState = ArmState.STOW_TO_SHELF;
          currAxis = CurrentAxis.ELBOW;
          elbowSubsystem.setPos(ArmState.SHELF.elbowPos);
          shoulderSubsystem.setPos(ArmState.SHELF.shoulderPos);
        }
        break;
      default:
        toStowPos(ArmState.SHELF);
        break;
    }
    desiredState = ArmState.SHELF;
  }

  public void toFloorPos() {
    hasElbowZeroed = false;
    toFloorPos(false);
  }

  public void toFloorPos(boolean continueToFloorSweep) {
    this.continueToFloorSweep = continueToFloorSweep;
    switch (currState) {
      case STOW:
        if (isElbowReinforced) {
          hasElbowZeroed = false;
          isElbowReinforced = false;
          elevatorSubsystem.unReinforceElevator();
          elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
        }
        if (!isElbowReinforced && elevatorSubsystem.isFinished()) {
          logger.info("{} -> STOW_TO_FLOOR", currState);
          currState = ArmState.STOW_TO_FLOOR;
          currAxis = CurrentAxis.ELBOW;
          elbowSubsystem.setPos(ArmState.FLOOR.elbowPos);
        }
        break;
      default:
        toStowPos(ArmState.FLOOR);
        break;
    }
    desiredState = ArmState.FLOOR;
  }

  public void toFloorSweep() {
    switch (currState) {
      case FLOOR:
        logger.info("{} -> FLOOR_TO_FLOOR_SWEEP", currState);
        currState = ArmState.FLOOR_TO_FLOOR_SWEEP;
        currAxis = CurrentAxis.ELBOW;
        logger.info("Set Elbow to {} Ticks", currState.elbowPos);
        elbowSubsystem.setPos(currState.elbowPos);
        break;
      default:
        toFloorPos();
    }
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

  public void stowShoulder() {
    currAxis = CurrentAxis.SHOULDER;
    shoulderSubsystem.zeroShoulder();
    ;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Hand X", () -> getHandPosition().getX()),
        new Measure("Hand Y", () -> getHandPosition().getY()),
        new Measure("Hand region", () -> getHandRegion().ordinal()),
        new Measure("Arm State", () -> currState.ordinal()));
  }

  public ArmState getCurrState() {
    return currState;
  }

  public void setArmFastStow(boolean setter) {
    shouldFastStowArm = setter;
    logger.info("Set Arm Fast Stow to: {}", setter);
  }

  public boolean isFastStowing() {
    return shouldFastStowArm;
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
        if (!hasElbowZeroed
            && !isElbowReinforced
            && desiredState == ArmState.STOW
            && doReinforceElevator) {
          elevatorSubsystem.reinforceElevator();
          elbowSubsystem.zeroElbowStow();
          hasElbowZeroed = true;
          isElbowReinforced = true;
        }
        switch (desiredState) {
          case LOW:
            toLowPos();
            break;
          case MID_CONE:
            toMidPos(GamePiece.CONE);
            break;
          case MID_CUBE:
            toMidPos(GamePiece.CUBE);
            break;
          case HIGH_CONE:
            toHighPos(GamePiece.CONE);
            break;
          case HIGH_CUBE:
            toHighPos(GamePiece.CUBE);
            break;
          case INTAKE_STAGE:
            toIntakePos();
            break;
          case INTAKE:
            toIntakeStagePos(continueToIntake);
            break;
          case SHELF:
            toShelfPos();
            break;
          case FLOOR_SWEEP:
            // fall through
          case FLOOR:
            toFloorPos();
            break;
          default:
            break;
        }
        break;
      case INTAKE_STAGE:
        if (continueToIntake) {
          toIntakePos();
        }

        break;
      case INTAKE:
        break;
      case LOW:
        break;
      case MID_CONE:
        break;
      case MID_CUBE:
        break;
      case HIGH_CONE:
        break;
      case HIGH_CUBE:
        break;
      case SHELF:
        break;
      case FLOOR:
        if (continueToFloorSweep) {
          toFloorSweep();
        }
        break;
      case MANUAL:
        break;
      case STOW_TO_INTAKE_STAGE:
        if (elbowSubsystem.isFinished()) {
          logger.info("{} -> INTAKE_STAGE", currState);
          currState = ArmState.INTAKE_STAGE;
          currAxis = CurrentAxis.NONE;
          logger.info("Continue to Intake: {}", continueToIntake);
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
              currAxis = CurrentAxis.NONE;
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
              elevatorSubsystem.setPos(desiredState.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              currAxis = CurrentAxis.SHOULDER;
              shoulderSubsystem.setPos(desiredState.shoulderPos);
            }
            break;
          case SHOULDER:
            if (shoulderSubsystem.isFinished()) {
              logger.info("{} -> {}", currState, desiredState);

              currState = desiredState;
              currAxis = CurrentAxis.NONE;
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
              elevatorSubsystem.setPos(desiredState.elevatorPos);
            }
            if (elbowSubsystem.getPos() >= ElbowConstants.kLevelTwoConeElbow && !isShoulderStaged) {
              isShoulderStaged = true;
              shoulderSubsystem.setPos(desiredState.shoulderPos);
              elevatorSubsystem.setPos(desiredState.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              currAxis = CurrentAxis.SHOULDER;

              if (!isShoulderStaged) shoulderSubsystem.setPos(desiredState.shoulderPos);
            }
            break;
          case SHOULDER:
            if (shoulderSubsystem.isFinished()) {
              logger.info("{} -> HIGH", currState);
              currState = desiredState;
              currAxis = CurrentAxis.NONE;
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
              currAxis = CurrentAxis.NONE;
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
              currAxis = CurrentAxis.NONE;
            }
            break;
          default:
            break;
        }
        break;
      case FLOOR_TO_STOW:
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
              currAxis = CurrentAxis.NONE;
            }
            break;

          default:
            break;
        }
        break;
      case SCORE_TO_STOW:
        switch (currAxis) {
          case SHOULDER:
            if (shouldFastStowArm) {
              isArmFastStowing = true;
              elbowSubsystem.setPos(ArmState.STOW.elbowPos);
              elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
            }
            if (shoulderSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELEVATOR;
              if (!isArmFastStowing) elevatorSubsystem.setPos(ArmState.STOW.elevatorPos);
            }
            break;
          case ELEVATOR:
            if (shouldFastStowArm && !isArmFastStowing) {
              isArmFastStowing = true;
              elbowSubsystem.setPos(ArmState.STOW.elbowPos);
            }
            if (elevatorSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELBOW;
              if (!isArmFastStowing) elbowSubsystem.setPos(ArmState.STOW.elbowPos);
            }
            break;
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              isArmFastStowing = false;
              logger.info("{} -> STOW", currState);
              currState = ArmState.STOW;
              currAxis = CurrentAxis.NONE;
            }
            break;

          default:
            break;
        }
        break;
      case INTAKE_STAGE_TO_INTAKE:
        switch (currAxis) {
          case ELEVATOR:
            if (elevatorSubsystem.isFinished()) {
              currAxis = CurrentAxis.ELBOW;
              elbowSubsystem.setPos(ArmState.INTAKE.elbowPos);
            }
            break;
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              currAxis = CurrentAxis.SHOULDER;
              shoulderSubsystem.setPos(ArmState.INTAKE.shoulderPos);
            }
            break;
          case SHOULDER:
            if (shoulderSubsystem.isFinished()) {
              logger.info("{} -> INTAKE", currState);
              currState = ArmState.INTAKE;
              currAxis = CurrentAxis.NONE;
            }
            break;
          default:
            break;
        }
        break;
      case INTAKE_TO_STOW:
        switch (currAxis) {
          case ELEVATOR:
          case SHOULDER: // Fall through
            if (elevatorSubsystem.isFinished() && shoulderSubsystem.isFinished()) {
              elbowSubsystem.setPos(ArmState.STOW.elbowPos);
              currAxis = CurrentAxis.ELBOW;
            }
            break;
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              logger.info("{} -> STOW", currState);
              currState = ArmState.STOW;
              currAxis = CurrentAxis.NONE;
            }
            break;
          default:
            break;
        }
        break;

      case SHELF_TO_STOW:
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
              currAxis = CurrentAxis.NONE;
            }
            break;

          default:
            break;
        }
        break;
      case FLOOR_TO_FLOOR_SWEEP:
        switch (currAxis) {
          case ELBOW:
            if (elbowSubsystem.isFinished()) {
              logger.info("{} -> FLOOR_SWEEP", currState);
              currState = ArmState.FLOOR_SWEEP;
              currAxis = CurrentAxis.NONE;
            }
        }
        break;
      case FLOOR_SWEEP:
        break;
      default:
        break;
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
        ElbowConstants.kIntakeStageElbow),
    LOW(
        ShoulderConstants.kLevelOneShoulder,
        ElevatorConstants.kLevelOneElevator,
        ElbowConstants.kLevelOneElbow),
    MID_CONE(
        ShoulderConstants.kLevelTwoConeShoulder,
        ElevatorConstants.kLevelTwoConeElevator,
        ElbowConstants.kLevelTwoConeElbow),
    MID_CUBE(
        ShoulderConstants.kLevelTwoCubeShoulder,
        ElevatorConstants.kLevelTwoCubeElevator,
        ElbowConstants.kLevelTwoCubeElbow),
    HIGH_CONE(
        ShoulderConstants.kLevelThreeConeShoulder,
        ElevatorConstants.kLevelThreeConeElevator,
        ElbowConstants.kLevelThreeConeElbow),
    HIGH_CUBE(
        ShoulderConstants.kLevelThreeCubeShoulder,
        ElevatorConstants.kLevelThreeCubeElevator,
        ElbowConstants.kLevelThreeCubeElbow),
    SHELF(
        ShoulderConstants.kShelfShoulder,
        ElevatorConstants.kShelfElevator,
        ElbowConstants.kShelfElbow),
    FLOOR(
        ShoulderConstants.kFloorShoulder,
        ElevatorConstants.kFloorElevator,
        ElbowConstants.kFloorElbow),
    MANUAL(0, 0, 0),
    STOW_TO_INTAKE_STAGE(INTAKE_STAGE),
    STOW_TO_LOW(LOW),
    STOW_TO_MID(MID_CONE), // Do not use for arm position
    STOW_TO_HIGH(HIGH_CONE), // Do not use for arm position
    STOW_TO_SHELF(SHELF),
    STOW_TO_FLOOR(FLOOR),
    INTAKE_STAGE_TO_INTAKE(INTAKE),
    INTAKE_TO_STOW(STOW),
    SCORE_TO_STOW(STOW),
    SHELF_TO_STOW(STOW),
    FLOOR_TO_STOW(STOW),
    MANUAL_TO_STOW(STOW),
    SCORE_STOW(STOW),
    FLOOR_SWEEP(
        ShoulderConstants.kFloorShoulder,
        ElevatorConstants.kFloorElevator,
        ElbowConstants.kFloorElbowSweep),
    FLOOR_TO_FLOOR_SWEEP(FLOOR_SWEEP);

    public final double shoulderPos;
    public final double elevatorPos;
    public final double elbowPos;

    ArmState(double shoulderPos, double elevatorPos, double elbowPos) {
      this.shoulderPos = shoulderPos;
      this.elevatorPos = elevatorPos;
      this.elbowPos = elbowPos;
    }

    ArmState(ArmState armState) {
      this.shoulderPos = armState.shoulderPos;
      this.elbowPos = armState.elbowPos;
      this.elevatorPos = armState.elevatorPos;
    }
  }

  public enum CurrentAxis {
    ELBOW,
    ELEVATOR,
    SHOULDER,
    NONE;
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
        ArmConstants.kElevatorBumperMin,
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
