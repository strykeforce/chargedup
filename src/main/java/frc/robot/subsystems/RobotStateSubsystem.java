package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HandConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem.DriveStates;
import frc.robot.subsystems.HandSubsystem.HandStates;
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
  private RGBlightsSubsystem rgbLightsSubsystem;
  private VisionSubsystem visionSubsystem;
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
  private Timer releaseDelayTimer = new Timer();
  private Timer intakeTimerOffset = new Timer();
  private boolean isReleaseDelayTimerRunning = false;
  private double currPoseX;
  private double desiredPoseX;
  private boolean isAutoStageFinished = false;
  private boolean isAutoPlacing = false;
  private boolean cameraWork = false;
  private boolean hasIntakeDelayPassed = false;
  private boolean fastStowAfterScore = false;
  private boolean allIntake = false;
  private double scorePosXIntial = -1.0;
  private Timer floorSweepTimer = new Timer();
  private boolean isAuto = false;
  private ElbowSubsystem elbowSubsystem;

  public RobotStateSubsystem(
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      RGBlightsSubsystem rgbLightsSubsystem,
      ElbowSubsystem elbowSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.armSubsystem = armSubsystem;
    this.handSubsystem = handSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.rgbLightsSubsystem = rgbLightsSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    logger.info("Serial Number: {}", RobotController.getSerialNumber());
  }

  public RobotState getRobotState() {
    return currRobotState;
  }

  public boolean isAutoPlaceFinished() {
    return isAutoStageFinished;
  }

  public void endAutoPlace(boolean interrupted) {
    isAutoStageFinished = false;
    logger.info("Finished Autoplace. Inturrupted: {}", interrupted);
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
    if (gamePiece == GamePiece.NONE) handSubsystem.runRollers(HandConstants.kRollerOff);
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

  public void setAutoMode(boolean isAuto) {
    this.isAuto = isAuto;
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Current State", () -> currRobotState.ordinal()));
  }

  public void setAllianceColor(Alliance alliance) {
    this.allianceColor = alliance;
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public boolean isCameraWorking() {
    return visionSubsystem.isCameraWorking();
  }

  public void toIntake() {
    logger.info("{} -->  TO_INTAKE_STAGE", currRobotState);
    currRobotState = RobotState.TO_INTAKE_STAGE;
    currentAxis = CurrentAxis.INTAKE;
    intakeSubsystem.startIntaking();
    if (handSubsystem.getHandState() == HandStates.OPEN
        || handSubsystem.getDesiredHandState() == HandStates.OPEN) {
      handSubsystem.stowHand(HandConstants.kCubeGrabbingPosition);
    }
  }

  public void toManualStage() {
    rgbLightsSubsystem.setOff();
    if (gamePiece != GamePiece.NONE) {
      if (currRobotState == RobotState.STOW) toManualScore();
      else toStowIntake(RobotState.MANUAL_SCORE);
    } else {
      if (currRobotState == RobotState.STOW) toManualShelf();
      else toStowIntake(RobotState.MANUAL_SHELF);
    }
  }

  public void toShelf() {
    if (currRobotState == RobotState.STOW) toManualShelf();
    else toStowIntake(RobotState.MANUAL_SHELF);
  }

  public void toAutoStage() {}

  public void toStowIntake() {
    toStowIntake(RobotState.STOW);
  }

  public void toStowIntake(RobotState nextState) {
    currRobotState = RobotState.TO_STOW_SCORE;
    if (elbowSubsystem.getPos() >= 0) {
      currRobotState = RobotState.TO_STOW_SCORE;
      logger.info("{} --> TO_STOW_SCORE", currRobotState);
      currentAxis = CurrentAxis.ARM;
      armSubsystem.toStowPos();
    } else if (gamePiece == GamePiece.NONE && elbowSubsystem.getPos() < 0) {
      logger.info("{} --> TO_STOW", currRobotState);
      currRobotState = RobotState.TO_STOW;

      switch (gamePiece) {
        case CONE:
          handSubsystem.grabCone();
          break;
        case CUBE:
          handSubsystem.grabCube();
          break;
        case NONE:
          handSubsystem.stowHand(HandConstants.kCubeGrabbingPosition);
      }
    }
    nextRobotState = nextState;
    currentAxis = CurrentAxis.HAND;
  }

  public void toStowScore(RobotState nextState) {
    logger.info("{} --> TO_STOW(SCORE)", currRobotState);
    currRobotState = RobotState.TO_STOW;
    nextRobotState = nextState;
    if (gamePiece == GamePiece.NONE) armSubsystem.toStowPos();
    currentAxis = CurrentAxis.ARM;
  }

  public void toFloorPickup() {
    logger.info("{} --> TO_FLOOR_PICKUP", currRobotState);
    if (currRobotState == RobotState.STOW) {
      handSubsystem.open();
      currRobotState = RobotState.TO_FLOOR_PICKUP;
      currentAxis = CurrentAxis.HAND;
    } else {
      toStowIntake(RobotState.FLOOR_PICKUP);
    }
  }

  public void toManualScore() {
    logger.info("{} --> TO_MANUAL_SCORE", currRobotState);

    switch (targetLevel) {
      case NONE:
        break;
      case LOW:
        if (armSubsystem.getCurrState() != ArmState.LOW) armSubsystem.toLowPos();
        break;
      case MID:
        if (armSubsystem.getCurrState() != ArmState.MID_CONE
            && armSubsystem.getCurrState() != ArmState.MID_CUBE) {
          armSubsystem.toMidPos(getGamePiece());
        }
        break;
      case HIGH:
        if (armSubsystem.getCurrState() != ArmState.HIGH_CONE
            && armSubsystem.getCurrState() != ArmState.HIGH_CUBE)
          armSubsystem.toHighPos(getGamePiece());
        break;
    }
    currRobotState = RobotState.TO_MANUAL_SCORE;
  }

  public void toManualShelf() {
    logger.info("{} --> TO_MANUAL_SHELF", currRobotState);
    if (armSubsystem.getCurrState() != ArmState.SHELF) armSubsystem.toShelfPos();
    currentAxis = CurrentAxis.ARM;
    currRobotState = RobotState.TO_MANUAL_SHELF;
  }

  public void toReleaseGamepiece() {
    logger.info("{} -> RELEASE_GAME_PIECE", currRobotState);
    currRobotState = RobotState.RELEASE_GAME_PIECE;
    handSubsystem.open();
    fastStowAfterScore = true;
    isReleaseDelayTimerRunning = false;
    releaseDelayTimer.stop();
    releaseDelayTimer.reset();
    scorePosXIntial = driveSubsystem.getPoseMeters().getX();
    handSubsystem.runRollers(HandConstants.kRollerDrop);
  }

  public void toGrabGamepiece(GamePiece gamePiece) {
    logger.info("{} -> GRAB_GAME_PIECE", currRobotState);
    currRobotState = RobotState.GRAB_GAME_PIECE;
    if (gamePiece == GamePiece.CONE) handSubsystem.grabCone();
    else handSubsystem.grabCube();
  }

  public void toAutoDrive() {
    logger.info("{} -> AUTO_DRIVE", currRobotState);
    currRobotState = RobotState.AUTO_DRIVE;
    isAutoPlacing = true;
    TargetCol tempTargetCol = TargetCol.NONE;
    if (Math.abs(driveSubsystem.getSpeedMPS()) <= DriveConstants.kMaxSpeedToAutoDrive) {
      if (gamePiece != GamePiece.CUBE) tempTargetCol = getTargetCol();
      logger.info("Gamepiece toAutodrive: {}", gamePiece.toString());
      driveSubsystem.autoDrive((gamePiece == GamePiece.NONE), tempTargetCol); // FIXME
    }
  }

  public void toAutoShelf() {
    logger.info("{} --> TO_AUTO_SHELF", currRobotState);
    if (armSubsystem.getCurrState() != ArmState.SHELF) armSubsystem.toShelfPos();
    isAutoPlacing = true; // FIXME
    currentAxis = CurrentAxis.ARM;
    currRobotState = RobotState.TO_AUTO_SHELF;
  }

  public void toAutoScore() {
    logger.info("{} --> AUTO_SCORE", currRobotState);
    // logger.info("ArmState: {}", armSubsystem.getCurrState());
    isAutoPlacing = true; // FIXME
    switch (targetLevel) {
      case NONE:
        break;
      case LOW:
        if (armSubsystem.getCurrState() != ArmState.LOW) armSubsystem.toLowPos();
        break;
      case MID:
        if (armSubsystem.getCurrState() != ArmState.MID_CONE
            && armSubsystem.getCurrState() != ArmState.MID_CUBE) {
          armSubsystem.toMidPos(getGamePiece());
        }
        break;
      case HIGH:
        if (armSubsystem.getCurrState() != ArmState.HIGH_CONE
            && armSubsystem.getCurrState() != ArmState.HIGH_CUBE)
          armSubsystem.toHighPos(getGamePiece());
        break;
    }
    currRobotState = RobotState.AUTO_SCORE;
  }

  public boolean shouldFastStowArm() {
    return (fastStowAfterScore
            && (isBlueAlliance()
                    && driveSubsystem.getPoseMeters().getX()
                        > (scorePosXIntial + RobotStateConstants.kRetakeAfterPlaceOffset)
                || (!isBlueAlliance()
                    && driveSubsystem.getPoseMeters().getX()
                        < (scorePosXIntial - RobotStateConstants.kRetakeAfterPlaceOffset))))
        || (fastStowAfterScore
            && isAuto
            && ((isBlueAlliance()
                    && driveSubsystem.getPoseMeters().getX() > AutonConstants.kMinXFastStow)
                || (!isBlueAlliance()
                    && driveSubsystem.getPoseMeters().getX()
                        < FieldConstants.kFieldLength - AutonConstants.kMinXFastStow)));
  }

  @Override
  public void periodic() {
    if (!cameraWork && isCameraWorking()) {
      rgbLightsSubsystem.setColor(0.0, 0.0, 0.0);
      cameraWork = true;
    }

    switch (currRobotState) {
      case STOW:
        if (currRobotState != nextRobotState) {
          switch (nextRobotState) {
            case INTAKE_STAGE:
              // fall-through
            case PICKUP_FROM_INTAKE:
              toIntake();
              break;
            case MANUAL_SCORE:
              toManualScore();
              break;
            case MANUAL_SHELF:
              toManualShelf();
              break;
            case FLOOR_PICKUP:
              toFloorPickup();
              break;
            case AUTO_SCORE: // not implemented
            case AUTO_SHELF: // not implemented
              break;
            case AUTO_DRIVE:
              break;
            default:
              break;
          }
        }
        break;
      case TO_AUTO_DRIVE:
        currRobotState = RobotState.AUTO_DRIVE;
        logger.info("{} -> AUTO_DRIVE", currRobotState);
        break;
      case TO_STOW_SCORE:
        switch (currentAxis) {
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.STOW) {
              if (gamePiece == GamePiece.NONE)
                handSubsystem.stowHand(HandConstants.kConeGrabbingPosition);
              currentAxis = CurrentAxis.HAND;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.INTAKE;
              intakeSubsystem.retractIntake();
            }
            break;
          case INTAKE:
            if (intakeSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              logger.info("{} -> STOW", currRobotState);
              currRobotState = RobotState.STOW;
            }
          default:
            break;
        }
        break;
      case TO_STOW:
        switch (currentAxis) {
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.ARM;
              if (shouldFastStowArm()) {
                armSubsystem.setArmFastStow(true);
              }
              armSubsystem.toStowPos();
            }
            break;
          case ARM:
            if (!armSubsystem.isFastStowing() && shouldFastStowArm()) {
              armSubsystem.setArmFastStow(true);
            }
            if (armSubsystem.getCurrState() == ArmState.STOW) {
              armSubsystem.setArmFastStow(false);
              fastStowAfterScore = false;
              if (isAuto) {
                currentAxis = CurrentAxis.NONE;
                logger.info("{} -> STOW", currRobotState);
                currRobotState = RobotState.STOW;
                intakeSubsystem.retractIntake(false);
                break;
              }
              currentAxis = CurrentAxis.INTAKE;
              intakeSubsystem.retractIntake();
            }

            break;
          case INTAKE:
            if (intakeSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              logger.info("{} -> STOW", currRobotState);
              currRobotState = RobotState.STOW;
            }
          default:
            break;
        }
        break;
      case TO_INTAKE_STAGE:
        switch (currentAxis) {
          case INTAKE:
            if (intakeSubsystem.isFinished()) {
              intakeTimerOffset.reset();
              intakeTimerOffset.start();
              currentAxis = CurrentAxis.ARM;
              break;
            }
            break;
          case ARM:
            if (intakeTimerOffset.hasElapsed(IntakeConstants.kIntakeDelay)
                && !hasIntakeDelayPassed) {
              hasIntakeDelayPassed = true;
              if (isAuto && shouldFastStowArm()) {
                armSubsystem.setArmFastStow(true);
              }
              armSubsystem.toIntakeStagePos(true);
            }
            if (armSubsystem.getCurrState() == ArmState.INTAKE) {
              hasIntakeDelayPassed = false;
              currentAxis = CurrentAxis.HAND;
              if (isAuto) armSubsystem.setArmFastStow(false);
              handSubsystem.openIntake();
              break;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              logger.info("{} -> INTAKE_STAGE", currRobotState);
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

        handSubsystem.runRollers(HandConstants.kRollerPickUp);
        if (handSubsystem.hasCube()) {
          logger.info("{} -> PICKUP_FROM_INTAKE", currRobotState);
          currRobotState = RobotState.PICKUP_FROM_INTAKE;
          intakeSubsystem.retractToPickupFromIntake();
          handSubsystem.grabCube();
          setGamePiece(GamePiece.CUBE);
          currentAxis = CurrentAxis.HAND;
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
              // armSubsystem.toIntakePos();
              // handSubsystem.grabCube();
              // currentAxis = CurrentAxis.HAND;
            }
            break;
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.INTAKE) {
              // handSubsystem.grabCube();
              // currentAxis = CurrentAxis.HAND;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished() || isAuto) {
              currentAxis = CurrentAxis.NONE;
              logger.info("Starting Intake Timer");
              intakeDelayTimer.reset();
              intakeDelayTimer.start();
            }
            break;
          case NONE:
            if (intakeDelayTimer.hasElapsed(IntakeConstants.kIntakePickupDelaySec)) {
              intakeDelayTimer.stop();
              handSubsystem.runRollers(HandConstants.kRollerCubeHoldSpeed);
              setGamePiece(GamePiece.CUBE);
              toStowIntake();
            }
            break;
          default:
            break;
        }

        break;
      case TO_MANUAL_SCORE:
        if (armSubsystem.getCurrState() == ArmState.LOW
            || armSubsystem.getCurrState() == ArmState.MID_CONE
            || armSubsystem.getCurrState() == ArmState.MID_CUBE
            || armSubsystem.getCurrState() == ArmState.HIGH_CONE
            || armSubsystem.getCurrState() == ArmState.HIGH_CUBE) {
          logger.info("{} -> MANUAL_SCORE", currRobotState);
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
              handSubsystem.openShelf();
              currentAxis = CurrentAxis.HAND;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              logger.info("{} -> MANUAL_SHELF", currRobotState);
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
        handSubsystem.runRollers(HandConstants.kRollerPickUp);
        if (handSubsystem.hasCone()) {
          handSubsystem.grabCone();
          gamePiece = GamePiece.CONE;
          currPoseX = driveSubsystem.getPoseMeters().getX();
          rgbLightsSubsystem.setColor(0.0, 1.0, 0.0);
          logger.info("{} -> SHELF_WAIT_TRANSITION", currRobotState);
          currRobotState = RobotState.SHELF_WAIT_TRANSITION;
        }

        break;
      case TO_AUTO_SCORE:
        if (armSubsystem.getCurrState() == ArmState.LOW
            || armSubsystem.getCurrState() == ArmState.MID_CONE
            || armSubsystem.getCurrState() == ArmState.MID_CUBE
            || armSubsystem.getCurrState() == ArmState.HIGH_CONE
            || armSubsystem.getCurrState() == ArmState.HIGH_CUBE) {
          logger.info("{} -> AUTO_SCORE", currRobotState);
          currRobotState = RobotState.AUTO_SCORE;
          currentAxis = CurrentAxis.NONE;
        }
      case AUTO_SCORE:
        // wait for arm to be in right position and then release the cone.
        isAutoStageFinished = true;
        toAutoDrive();
        break;
      case TO_AUTO_SHELF:
        switch (currentAxis) {
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.SHELF) {
              handSubsystem.openShelf();
              currentAxis = CurrentAxis.HAND;
              logger.info("TO_AUTO_SHELF: arm finished");
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              isAutoStageFinished = true;
              logger.info("TO_AUTO_SHELF: hand finished");
              toAutoDrive();
              // logger.info("{} -> AUTO_SHELF", currRobotState);
              // currRobotState = RobotState.AUTO_SHELF;
            }
            break;
          default:
            break;
        }
        break;
      case AUTO_SHELF:
        handSubsystem.runRollers(HandConstants.kRollerPickUp);
        if (handSubsystem.hasCone()) {
          handSubsystem.grabCone();
          gamePiece = GamePiece.CONE;
          currPoseX = driveSubsystem.getPoseMeters().getX();
          rgbLightsSubsystem.setColor(0.0, 1.0, 0.0);
          logger.info("{} -> SHELF_WAIT_TRANSITION", currRobotState);
          currRobotState = RobotState.SHELF_WAIT_TRANSITION;
        }
        break;

      case TO_FLOOR_PICKUP:
        switch (currentAxis) {
          case HAND:
            if (handSubsystem.isFinished()) {
              armSubsystem.toFloorPos();
              handSubsystem.runRollers(HandConstants.kRollerPickUp);
              currentAxis = CurrentAxis.ARM;
            }
            break;
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.FLOOR) {
              currentAxis = CurrentAxis.NONE;
              logger.info("{} -> FLOOR_GRAB_CONE", currRobotState);
              currRobotState = RobotState.FLOOR_GRAB_CONE;
              handSubsystem.openFloor();
              floorSweepTimer.reset();
              floorSweepTimer.start();
            }
            break;
          default:
            break;
        }
        break;
      case FLOOR_GRAB_CONE:
        switch (currentAxis) {
          case NONE:
            if (floorSweepTimer.hasElapsed(ArmConstants.kSweepTimerElapseSeconds)) {
              logger.info("FloorSweepTimer Elapsed");
              floorSweepTimer.reset();
              armSubsystem.toFloorSweep();
              currentAxis = CurrentAxis.ARM;
            }
            break;
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.FLOOR_SWEEP) {
              handSubsystem.grabCone();
              setGamePiece(GamePiece.CONE);
              currentAxis = CurrentAxis.HAND;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) toStowIntake();
            break;
        }
        break;
      case FLOOR_PICKUP:
        handSubsystem.runRollers(HandConstants.kRollerPickUp);
        break;

      case RELEASE_GAME_PIECE:
        // (from) MANUAL_SCORE or AUTO_SCORE
        // open hand
        // (break)

        if (handSubsystem.isFinished() && !isReleaseDelayTimerRunning) {
          rgbLightsSubsystem.setColor(0.0, 0.0, 0.0);
          setGamePiece(GamePiece.NONE);
          releaseDelayTimer.reset();
          releaseDelayTimer.start();
          isReleaseDelayTimerRunning = true;
          logger.info("Started release timer");
        } else if (isReleaseDelayTimerRunning
            && releaseDelayTimer.hasElapsed(RobotStateConstants.kReleaseDelayTime)) {

          isReleaseDelayTimerRunning = false;
          releaseDelayTimer.stop();
          releaseDelayTimer.reset();
          logger.info("Release timer elapsed.");
          toStowIntake();
        }

        break;
      case GRAB_GAME_PIECE:
        if (handSubsystem.isFinished()) {
          if (handSubsystem.getHandState() == HandStates.CONE_CLOSED) setGamePiece(GamePiece.CONE);
          else setGamePiece(GamePiece.CUBE);
          toStowIntake();
        }
        break;
      case SHELF_WAIT_TRANSITION:
        if (allianceColor == Alliance.Blue) {
          desiredPoseX = currPoseX - Constants.ArmConstants.kShelfTransitionMove;
          if (driveSubsystem.getPoseMeters().getX() <= desiredPoseX) {
            logger.info("{} -> SHELF_WAIT", currRobotState);
            currRobotState = RobotState.SHELF_WAIT;
          }
        } else if (allianceColor == Alliance.Red) {
          desiredPoseX = currPoseX + Constants.ArmConstants.kShelfTransitionMove;
          if (driveSubsystem.getPoseMeters().getX() >= desiredPoseX) {
            logger.info("SHELF_WAIT_TRANSITION -> SHELF_WAIT");
            currRobotState = RobotState.SHELF_WAIT;
          }
        }

        break;
      case SHELF_WAIT:
        // (from) AUTO_SHELF or MANUAL_SHELF
        // check if robot has moved enough
        // (to) STOW

        if (driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FINISHED) {
          logger.info("{} -> IDLE", driveSubsystem.currDriveState);
          driveSubsystem.setDriveState(DriveStates.IDLE);
        }

        // FIXME IF FROM AUTO_SHELF AND IF SHELF INCLUDES GRABBING THE GAMEPIECE(IDK I
        // DIDNT READ
        // IT), THEN END THE AUTOPLACE COMMAND

        if (allianceColor == Alliance.Blue) {
          desiredPoseX = currPoseX - Constants.ArmConstants.kShelfMove;
          if (driveSubsystem.getPoseMeters().getX() <= desiredPoseX) {
            rgbLightsSubsystem.setOff();
            toStowIntake();
          }
        } else if (allianceColor == Alliance.Red) {
          desiredPoseX = currPoseX + Constants.ArmConstants.kShelfMove;
          if (driveSubsystem.getPoseMeters().getX() >= desiredPoseX) {
            rgbLightsSubsystem.setOff();
            toStowIntake();
          }
        }

        break;
      case AUTO_DRIVE:
        if (driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FAILED) {
          if (gamePiece == GamePiece.NONE) {
            handSubsystem.runRollers(HandConstants.kRollerPickUp);
          }
          rgbLightsSubsystem.setColor(1.0, 0.0, 0.0);
          logger.info("{} -> CHECK_AMBIGUITY", currRobotState);
          currRobotState = RobotState.CHECK_AMBIGUITY;
        } else {
          rgbLightsSubsystem.setColor(0.0, 1.0, 1.0);
        }
        if (driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FINISHED) {
          // logger.info("Set RGB OFF");
          rgbLightsSubsystem.setOff();
          // driveSubsystem.currDriveState = DriveStates.IDLE;
          // Start Arm Stuff.
          isAutoPlacing = false;
          // isAutoStageFinished = true;
          // if (gamePiece == GamePiece.NONE) toAutoShelf();
          // else toAutoScore();
          if (gamePiece == GamePiece.NONE) {
            logger.info("{} -> AUTO_SHELF", currRobotState);
            currRobotState = RobotState.AUTO_SHELF;
          }
        } // FIXME ELSE??
        break;
      case CHECK_AMBIGUITY:
        if (gamePiece == GamePiece.NONE && handSubsystem.hasCone()) {
          handSubsystem.grabCone();
          gamePiece = GamePiece.CONE;
          currPoseX = driveSubsystem.getPoseMeters().getX();
          rgbLightsSubsystem.setColor(0.0, 1.0, 0.0);
          logger.info("{} -> SHELF_WAIT_TRANSITION", currRobotState);
          currRobotState = RobotState.SHELF_WAIT_TRANSITION;
        }
        if (visionSubsystem.getAmbiguity() <= 0.15) {
          rgbLightsSubsystem.setColor(0.0, 1.0, 1.0);
          // toAutoDrive();
        }
        if (visionSubsystem.getAmbiguity() > 0.15) {
          rgbLightsSubsystem.setColor(1.0, 0.0, 0.0);
        }
        break;
      default:
        break;
    }
  }

  public Pose2d getShelfPosAutoDrive(TargetCol tempTargetCol, boolean isBlue) {
    int multiplier = 0;
    logger.info("tempTargetCol: {}", tempTargetCol.name());
    if (tempTargetCol.equals(TargetCol.LEFT)) multiplier = -1;
    if (tempTargetCol.equals(TargetCol.RIGHT)) multiplier = 1;
    if (isBlue) multiplier *= -1;

    if (isBlue)
      return new Pose2d(
          new Translation2d(
              RobotStateConstants.kShelfBlue.getX(),
              RobotStateConstants.kShelfBlue.getY()
                  + RobotStateConstants.kShelfOffset * multiplier),
          new Rotation2d(0.0));
    return new Pose2d(
        new Translation2d(
            RobotStateConstants.kShelfRed.getX(),
            RobotStateConstants.kShelfRed.getY() + RobotStateConstants.kShelfOffset * multiplier),
        new Rotation2d(Math.PI));
  }

  public Pose2d getAutoPlaceDriveTarget(double yCoord, TargetCol tempTargetCol) {
    int gridIndex =
        ((yCoord > Constants.RobotStateConstants.kBound1Y) ? 1 : 0)
            + ((yCoord > Constants.RobotStateConstants.kBound2Y) ? 1 : 0);
    double targetX = Constants.RobotStateConstants.kAutoPlaceX;
    double rotation = Math.PI;
    if (!isBlueAlliance()) rotation = 0.0;

    if (!isBlueAlliance()) {
      targetX = Constants.RobotStateConstants.kFieldMaxX - targetX;
      logger.info("Not blue alliance: {}", allianceColor.toString());
      rotation = 0;
    }
    int multiplier = 0;
    logger.info("tempTargetCol: {}", tempTargetCol.name());
    if (tempTargetCol.equals(TargetCol.LEFT)) multiplier = 1;
    if (tempTargetCol.equals(TargetCol.RIGHT)) multiplier = -1;
    if (!isBlueAlliance()) multiplier *= -1;
    return new Pose2d(
        targetX,
        Constants.RobotStateConstants.kGridY[gridIndex]
            + multiplier * RobotStateConstants.kPolePlaceOffset,
        new Rotation2d(rotation));
  }

  public boolean isBlueAlliance() {
    return getAllianceColor() == Alliance.Blue;
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
    SHELF_WAIT_TRANSITION,
    SHELF_WAIT,
    ARM_TRANSITION,
    TO_MANUAL_SCORE,
    TO_MANUAL_SHELF,
    TO_FLOOR_PICKUP,
    GRAB_GAME_PIECE,
    TO_STOW,
    TO_AUTO_DRIVE,
    AUTO_DRIVE,
    TO_AUTO_SHELF,
    TO_AUTO_SCORE,
    CHECK_AMBIGUITY,
    FLOOR_GRAB_CONE,
    TO_STOW_SCORE
  }

  public enum CurrentAxis {
    HAND,
    ARM,
    INTAKE,
    ARM_AND_INTAKE,
    NONE;
  }
}
