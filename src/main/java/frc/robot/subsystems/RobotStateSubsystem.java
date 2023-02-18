package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.HandConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  private IntakeSubsystem intakeSubsystem;
  private ArmSubsystem armSubsystem;
  private HandSubsystem handSubsystem;
  private DriveSubsystem driveSubsystem;
  private TargetLevel targetLevel = TargetLevel.NONE;
  private TargetCol targetCol = TargetCol.NONE;
  private GamePiece gamePiece = GamePiece.NONE;
  private RobotState currRobotState = RobotState.STOW;
  private RobotState nextRobotState = RobotState.STOW;
  private CurrentAxis currentAxis = CurrentAxis.NONE;
  private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);
  private Alliance allianceColor = DriverStation.getAlliance();
  private boolean isAutoStage;
  private Timer intakeDelayTimer = new Timer();
  private boolean isIntakeTimerRunning = false;
  private double currPoseX;
  private double desiredPoseX;

  public RobotStateSubsystem(
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      DriveSubsystem driveSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.handSubsystem = handSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  public RobotState getRobotState() {
    return currRobotState;
  }

  public void setTargetLevel(TargetLevel targetLevel) {
    this.targetLevel = targetLevel;
    logger.info("set targetLevel to: {}", targetLevel);
  }

  public TargetLevel getTargetLevel() {
    return targetLevel;
  }

  public void setTargetCol(TargetCol targetCol) {
    this.targetCol = targetCol;
    logger.info("set targetCol to: {}", targetCol);
  }

  public TargetCol getTargetCol() {
    return targetCol;
  }

  public void setGamePiece(GamePiece gamePiece) {
    this.gamePiece = gamePiece;
    logger.info("set gamePiece to: {}", gamePiece);
  }

  public GamePiece getGamePiece() {
    return gamePiece;
  }

  public boolean isAutoStaging() {
    return isAutoStage;
  }

  public void setAutoStaging(boolean enable) {
    this.isAutoStage = enable;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public void setAllianceColor(Alliance alliance) {
    this.allianceColor = alliance;
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public void toIntake() {
    logger.info("{} -->  TO_INTAKE_STAGE", currRobotState);
    currRobotState = RobotState.TO_INTAKE_STAGE;
    currentAxis = CurrentAxis.ARM_AND_INTAKE;
    intakeSubsystem.startIntaking();
    armSubsystem.toIntakeStagePos();
  }

  public void toMaunalStage() {
    if (handSubsystem.hasPiece()) {
      toStow(RobotState.MANUAL_SCORE);
    } else {
      toStow(RobotState.MANUAL_SHELF);
    }
  }

  public void toAutoStage() {}

  public void toStow() {
    intakeSubsystem.retractIntake();

    if (gamePiece == GamePiece.NONE) handSubsystem.setPos(HandConstants.kConeGrabbingPosition);
    currentAxis = CurrentAxis.HAND;

    toStow(RobotState.STOW);
  }

  public void toStow(RobotState nextState) {
    logger.info("{} --> STOW", currRobotState);

    currRobotState = RobotState.TO_STOW;
    nextRobotState = nextState;
  }

  public void toFloorPickup() {
    logger.info("{} --> TO_FLOOR_PICKUP", currRobotState);

    armSubsystem.toFloorPos();
    currRobotState = RobotState.TO_FLOOR_PICKUP;
    currentAxis = CurrentAxis.ARM;
  }

  public void toManualScore() {
    logger.info("{} --> MANUAL_SCORE, currRobotState");

    switch (targetLevel) {
      case NONE:
        break;
      case LOW:
        armSubsystem.toLowPos();
        break;
      case MID:
        armSubsystem.toMidPos();
        break;
      case HIGH:
        armSubsystem.toHighPos();
        break;
    }
    currRobotState = RobotState.TO_MANUAL_SCORE;
  }

  public void toManualShelf() {
    logger.info("{} --> TO_MANUAL_SHELF, currRobotState");
    armSubsystem.toShelfPos();
    currentAxis = CurrentAxis.ARM;
    currRobotState = RobotState.TO_MANUAL_SHELF;
  }

  @Override
  public void periodic() {
    switch (currRobotState) {
      case STOW:
        currRobotState = nextRobotState;
        break;

      case TO_STOW:
        switch (currentAxis) {
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.ARM;
              armSubsystem.toStowPos();
            }
            break;
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.STOW) {
              currentAxis = CurrentAxis.NONE;
              currRobotState = RobotState.STOW;
            }
          default:
            break;
        }
        break;

      case TO_INTAKE_STAGE:
        switch (currentAxis) {
          case ARM_AND_INTAKE:
            if (intakeSubsystem.isFinished()
                && armSubsystem.getCurrState() == ArmState.INTAKE_STAGE) {
              handSubsystem.open();
              currentAxis = CurrentAxis.HAND;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              currRobotState = RobotState.INTAKE_STAGE;
            }
            break;
          default:
            break;
        }
        break;

      case INTAKE_STAGE:
        // (from) TO_INTAKE_STAGE
        // (wait) beam break
        // (to) PICKUP_FROM_INTAKE

        if (intakeSubsystem.getIsBeamBreakActive()) {
          currRobotState = RobotState.PICKUP_FROM_INTAKE;
          intakeSubsystem.retractIntake();
        }

        break;

      case PICKUP_FROM_INTAKE:
        // (from) INTAKE_STAGE
        // (wait) intake retract
        // (wait) elevator down
        // (wait) hand close
        // (wait) small delay
        // set game piece cube
        // (to) STOW

        switch (currentAxis) {
          case INTAKE:
            if (intakeSubsystem.isFinished()) {
              armSubsystem.toIntakePos();
              currentAxis = CurrentAxis.ARM;
            }
            break;
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.INTAKE) {
              handSubsystem.grabCube();
              currentAxis = CurrentAxis.HAND;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              if (!isIntakeTimerRunning) {
                isIntakeTimerRunning = true;
                intakeDelayTimer.reset();
                intakeDelayTimer.start();
              }
              if (!intakeDelayTimer.hasElapsed(IntakeConstants.kIntakePickupDelaySec)) break;
              intakeDelayTimer.stop();
              isIntakeTimerRunning = false;

              setGamePiece(GamePiece.CUBE);

              toStow();
            }
            break;
          default:
            break;
        }

        break;
      case TO_MANUAL_SCORE:
        if (armSubsystem.getCurrState() == ArmState.LOW
            || armSubsystem.getCurrState() == ArmState.MID
            || armSubsystem.getCurrState() == ArmState.HIGH) {
          currRobotState = RobotState.MANUAL_SCORE;
          currentAxis = CurrentAxis.NONE;
        }
        break;

      case MANUAL_SCORE:
        break;

      case TO_MANUAL_SHELF:
        switch (currentAxis) {
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.SHELF) {
              handSubsystem.open();
              currentAxis = CurrentAxis.HAND;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              currRobotState = RobotState.MANUAL_SHELF;
            }
            break;
          default:
            break;
        }

        break;

      case MANUAL_SHELF:
        // (from) TO_MANUAL_SHELF
        // grab game piece
        // (to) SHELF_WAIT

        if (handSubsystem.hasPiece()) {
          handSubsystem.grabCone();
          currPoseX = driveSubsystem.getPoseMeters().getX();
          currRobotState = RobotState.SHELF_WAIT;
        }

        break;
      case AUTO_SCORE:
        break;
      case AUTO_SHELF:
        break;

      case TO_FLOOR_PICKUP:
        if (armSubsystem.getCurrState() == ArmState.FLOOR) {
          currentAxis = CurrentAxis.NONE;
          currRobotState = RobotState.FLOOR_PICKUP;
        }
        break;

      case FLOOR_PICKUP:
        break;

      case RELEASE_GAME_PIECE:
        // (from) MANUAL_SCORE or AUTO_SCORE
        // open hand
        // (break)

        handSubsystem.setPos(Constants.HandConstants.kHandOpenPosition);

        break;

      case SHELF_WAIT:
        // (from) AUTO_SHELF or MANUAL_SHELF
        // check if robot has moved enough
        // (to) STOW

        if (allianceColor == Alliance.Blue) {
          desiredPoseX = currPoseX + Constants.ArmConstants.kShelfMove;
          if (driveSubsystem.getPoseMeters().getX() >= desiredPoseX) {
            toStow();
          }
        } else if (allianceColor == Alliance.Red) {
          desiredPoseX = currPoseX - Constants.ArmConstants.kShelfMove;
          if (driveSubsystem.getPoseMeters().getX() <= desiredPoseX) {
            toStow();
          }
        }

        break;

      default:
        break;
    }
  }

  public enum TargetLevel {
    NONE,
    LOW,
    MID,
    HIGH
  };

  public enum TargetCol {
    NONE,
    LEFT,
    MID,
    RIGHT
  };

  public enum GamePiece {
    NONE,
    CUBE,
    CONE
  };

  public enum RobotState {
    STOW,
    TO_INTAKE_STAGE,
    INTAKE_STAGE,
    PICKUP_FROM_INTAKE,
    MANUAL_SCORE,
    MANUAL_SHELF,
    AUTO_SCORE,
    AUTO_SHELF,
    FLOOR_PICKUP,
    RELEASE_GAME_PIECE,
    SHELF_WAIT,
    ARM_TRANSITION,
    TO_MANUAL_SCORE,
    TO_MANUAL_SHELF,
    TO_FLOOR_PICKUP,
    TO_STOW
  }

  public enum CurrentAxis {
    HAND,
    ARM,
    INTAKE,
    ARM_AND_INTAKE,
    NONE;
  }
}
