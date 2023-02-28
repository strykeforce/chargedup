package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HandConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.commands.hand.HandLeftSpeedCommand;
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
  private boolean isReleaseDelayTimerRunning = false;
  private double currPoseX;
  private double desiredPoseX;
  private boolean isAutoStageFinished = false;
  private boolean isAutoPlacing = false;
  private boolean cameraWork = false;

  public RobotStateSubsystem(
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      RGBlightsSubsystem rgbLightsSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.armSubsystem = armSubsystem;
    this.handSubsystem = handSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.rgbLightsSubsystem = rgbLightsSubsystem;
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
    if (gamePiece == GamePiece.NONE)
      handSubsystem.runRollers(HandConstants.kRollerOff);
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
    currentAxis = CurrentAxis.ARM_AND_INTAKE;
    intakeSubsystem.startIntaking();
    armSubsystem.toIntakeStagePos();
  }

  public void toManualStage() {
    rgbLightsSubsystem.setOff();
    if (gamePiece != GamePiece.NONE) {
      if (currRobotState == RobotState.STOW) toManualScore();
      else toStow(RobotState.MANUAL_SCORE);
    } else {
      if (currRobotState == RobotState.STOW) toManualShelf();
      else toStow(RobotState.MANUAL_SHELF);
    }
  }

  public void toShelf() {
    if (currRobotState == RobotState.STOW) toManualShelf();
    else toStow(RobotState.MANUAL_SHELF);
  }

  public void toAutoStage() {}

  public void toStow() {
    toStow(RobotState.STOW);
  }

  public void toStow(RobotState nextState) {
    logger.info("{} --> TO_STOW", currRobotState);
    currRobotState = RobotState.TO_STOW;
    nextRobotState = nextState;

    intakeSubsystem.retractIntake();

    if (gamePiece == GamePiece.NONE) handSubsystem.setPos(HandConstants.kConeGrabbingPosition);
    currentAxis = CurrentAxis.HAND;
  }

  public void toFloorPickup() {
    logger.info("{} --> TO_FLOOR_PICKUP", currRobotState);
    if (currRobotState == RobotState.STOW) {
      handSubsystem.open();
      currRobotState = RobotState.TO_FLOOR_PICKUP;
      currentAxis = CurrentAxis.HAND;
    } else {
      toStow(RobotState.FLOOR_PICKUP);
    }
  }

  public void toManualScore() {
    logger.info("{} --> TO_MANUAL_SCORE", currRobotState);

    switch (targetLevel) {
      case NONE:
        break;
      case LOW:
        armSubsystem.toLowPos();
        break;
      case MID:
        armSubsystem.toMidPos(getGamePiece());
        break;
      case HIGH:
        armSubsystem.toHighPos(getGamePiece());
        break;
    }
    currRobotState = RobotState.TO_MANUAL_SCORE;
  }

  public void toManualShelf() {
    logger.info("{} --> TO_MANUAL_SHELF", currRobotState);
    armSubsystem.toShelfPos();
    currentAxis = CurrentAxis.ARM;
    currRobotState = RobotState.TO_MANUAL_SHELF;
  }

  public void toReleaseGamepiece() {
    logger.info("{} -> RELEASE_GAME_PIECE", currRobotState);
    currRobotState = RobotState.RELEASE_GAME_PIECE;
    handSubsystem.open();
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
    armSubsystem.toShelfPos();
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
              logger.info("{} -> STOW", currRobotState);
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
              handSubsystem.openIntake();
              currentAxis = CurrentAxis.HAND;
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

        if (intakeSubsystem.isBeamBroken()) {
          logger.info("{} -> PICKUP_FROM_INTAKE", currRobotState);
          currRobotState = RobotState.PICKUP_FROM_INTAKE;
          intakeSubsystem.retractIntake();
          currentAxis = CurrentAxis.INTAKE;
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
              handSubsystem.runRollers(HandConstants.kRollerPickUp);
              currentAxis = CurrentAxis.NONE;
              logger.info("Starting Intake Timer");
              intakeDelayTimer.reset();
              intakeDelayTimer.start();
            }
            break;
          case NONE:
            if (intakeDelayTimer.hasElapsed(IntakeConstants.kIntakePickupDelaySec)) {
              intakeDelayTimer.stop();
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
              handSubsystem.open();
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
        if (handSubsystem.hasPiece()) {
          handSubsystem.grabCone();
          gamePiece = GamePiece.CONE;
          currPoseX = driveSubsystem.getPoseMeters().getX();
          currRobotState = RobotState.SHELF_WAIT;
          rgbLightsSubsystem.setColor(0.0, 1.0, 0.0);
          ;
          logger.info("{} -> SHELF_WAIT", currRobotState);
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
              handSubsystem.open();
              currentAxis = CurrentAxis.HAND;
            }
            break;
          case HAND:
            if (handSubsystem.isFinished()) {
              currentAxis = CurrentAxis.NONE;
              isAutoStageFinished = true;
              toAutoDrive();
              // logger.info("{} -> AUTO_SHELF", currRobotState);
              // currRobotState = RobotState.AUTO_SHELF;
            }
            break;
          default:
            break;
        }
      case AUTO_SHELF:
        handSubsystem.runRollers(HandConstants.kRollerPickUp);
        if (handSubsystem.hasPiece()) {
          handSubsystem.grabCone();
          gamePiece = GamePiece.CONE;
          currPoseX = driveSubsystem.getPoseMeters().getX();
          rgbLightsSubsystem.setColor(0.0, 1.0, 0.0);
          logger.info("{} -> SHELF_WAIT", currRobotState);
          currRobotState = RobotState.SHELF_WAIT;
        }
        break;

      case TO_FLOOR_PICKUP:
        switch (currentAxis) {
          case HAND:
            if (handSubsystem.isFinished()) {
              armSubsystem.toFloorPos();
              currentAxis = CurrentAxis.ARM;
            }
            break;
          case ARM:
            if (armSubsystem.getCurrState() == ArmState.FLOOR) {
              currentAxis = CurrentAxis.NONE;
              logger.info("{} -> FLOOR_PICKUP", currRobotState);
              currRobotState = RobotState.FLOOR_PICKUP;
            }
            break;
          default:
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
          setGamePiece(GamePiece.NONE);
          releaseDelayTimer.reset();
          releaseDelayTimer.start();
          isReleaseDelayTimerRunning = true;
        } else if (isReleaseDelayTimerRunning
            && releaseDelayTimer.hasElapsed(RobotStateConstants.kReleaseDelayTime)) {
          isReleaseDelayTimerRunning = false;
          releaseDelayTimer.stop();
          toStow();
        }

        break;
      case GRAB_GAME_PIECE:
        if (handSubsystem.isFinished()) {
          if (handSubsystem.getHandState() == HandStates.CONE_CLOSED) setGamePiece(GamePiece.CONE);
          else setGamePiece(GamePiece.CUBE);
          toStow();
        }
        break;
      case SHELF_WAIT:
        // (from) AUTO_SHELF or MANUAL_SHELF
        // check if robot has moved enough
        // (to) STOW
        if (driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FINISHED) {
          logger.info("DRIVESUB: {} -> IDLE", driveSubsystem.currDriveState);
          driveSubsystem.setDriveState(DriveStates.IDLE);
        }

        // FIXME IF FROM AUTO_SHELF AND IF SHELF INCLUDES GRABBING THE GAMEPIECE(IDK I DIDNT READ
        // IT), THEN END THE AUTOPLACE COMMAND

        if (allianceColor == Alliance.Blue) {
          desiredPoseX = currPoseX - Constants.ArmConstants.kShelfMove;
          if (driveSubsystem.getPoseMeters().getX() <= desiredPoseX) {
            rgbLightsSubsystem.setOff();
            toStow();
          }
        } else if (allianceColor == Alliance.Red) {
          desiredPoseX = currPoseX + Constants.ArmConstants.kShelfMove;
          if (driveSubsystem.getPoseMeters().getX() >= desiredPoseX) {
            rgbLightsSubsystem.setOff();
            toStow();
          }
        }

        break;
      case AUTO_DRIVE:
        if (driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FAILED) {
          rgbLightsSubsystem.setColor(1.0, 0.0, 0.0);
          logger.info("ROBOT STATE: {} -> CHECK_AMBIGUITY", currRobotState);
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
    CHECK_AMBIGUITY
  }

  public enum CurrentAxis {
    HAND,
    ARM,
    INTAKE,
    ARM_AND_INTAKE,
    NONE;
  }
}
