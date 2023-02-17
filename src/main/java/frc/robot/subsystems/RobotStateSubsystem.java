package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);
  private Alliance allianceColor = DriverStation.getAlliance();
  private boolean isAutoStage;
  private Timer intakeDelayTimer = new Timer();
  private boolean isIntakeTimerRunning = false;
  private double currPoseX;
  private double desiredPoseX;

  public RobotStateSubsystem(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem, DriveSubsystem driveSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.handSubsystem = handSubsystem;
    this.driveSubsystem = driveSubsystem;
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

  public void toIntake(){
    logger.info("{} -->  TO_INTAKE_STAGE", currRobotState);
      currRobotState = RobotState.TO_INTAKE_STAGE;
        intakeSubsystem.startIntaking();
          armSubsystem.toIntakeStagePos();
  }

  public void toMaunalStage() {
    // if (handSubsystem.hasGamePiece()) (aka. manual score)
    toStow(RobotState.MANUAL_SCORE);
    // else (aka. manual shelf)
    toStow(RobotState.MANUAL_SHELF);
  }

  public void toAutoStage() {
    
  }

  public void toStow() {
    intakeSubsystem.retractIntake();
    if (gamePiece == GamePiece.NONE) handSubsystem.setPos(HandConstants.kConeGrabbingPosition);
    armSubsystem.toStowPos();
    toStow(RobotState.STOW);
  }

  public void toStow(RobotState nextState) {
    currRobotState = RobotState.STOW;
    nextRobotState = nextState;
  }

  @Override
  public void periodic() {
    switch (currRobotState) {
      case STOW:
        // retract intake
        // (wait) close hand
        // (wait) pull arm in

        intakeSubsystem.retractIntake();

        if (gamePiece == GamePiece.NONE) {
          handSubsystem.setPos(HandConstants.kConeGrabbingPosition);
          if (!handSubsystem.isFinished()) break;
        }

        armSubsystem.toStowPos();
        if (armSubsystem.getCurrState() != ArmState.STOW) break;

        currRobotState = nextRobotState;

        break;
      case TO_INTAKE_STAGE:
        // (from) STOW
        // start intaking
        // (wait) elbow to position
        // (wait) open hand
        // (to) INTAKE_STAGE

        intakeSubsystem.startIntaking();

        armSubsystem.toIntakeStagePos();
        if (!armSubsystem.isFinished()) break; //FIXME

        handSubsystem.setPos(HandConstants.kHandOpenPosition);
        if (!handSubsystem.isFinished()) break;

        currRobotState = RobotState.INTAKE_STAGE;
      case INTAKE_STAGE:
        // (from) TO_INTAKE_STAGE
        // (wait) beam break
        // (to) INTAKE

        if (!intakeSubsystem.getIsBeamBreakActive()) break;

        currRobotState = RobotState.INTAKE;
      case INTAKE:
        // (from) INTAKE_STAGE
        // (wait) intake retract
        // (wait) elevator down
        // (wait) hand close
        // (wait) small delay
        // set game piece cube
        // (to) STOW

        intakeSubsystem.retractIntake();
        if (!intakeSubsystem.isFinished()) break;

        armSubsystem.toIntakePos();
        if (!armSubsystem.isFinished()) break;

        handSubsystem.grabCube();
        if (!handSubsystem.isFinished()) break;

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

        break;
      case MANUAL_SCORE:
        // (from) STOW
        // arm to correct height
        // (break) button to RELEASE_GAME_PIECE

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

        break;
      case MANUAL_SHELF:
        // (from) STOW
        // arm to correct height
        // grab game piece
        // (to) SHELF_WAIT

        armSubsystem.toShelfPos();
        // TODO check proximity sensor
        handSubsystem.grabCone();
        currRobotState = RobotState.SHELF_WAIT;

        break;
      case AUTO_SCORE:
        break;
      case AUTO_SHELF:
        break;
      case FLOOR_PICKUP:
        // (from) STOW
        // arm to correct height
        // grab game piece
        // (to) STOW

        armSubsystem.toFloorPos();
        if (armSubsystem.getCurrState() == ArmState.FLOOR) {
            handSubsystem.grabCone();
            gamePiece = GamePiece.CONE;
          if (handSubsystem.isFinished()) {
            toStow();
          }
        }

        break;
      case RELEASE_GAME_PIECE:
        // (from) MANUAL_SCORE or AUTO_SCORE
        // open hand
        // (break)

        handSubsystem.setPos(Constants.HandConstants.kHandOpenPosition);

        break;
      default:
        break;
      case SHELF_WAIT:
        // (from) AUTO_SHELF or MANUAL_SHELF
        // check if robot has moved enough
        // (to) STOW

        currPoseX = driveSubsystem.getPoseMeters().getX();
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

  private enum RobotState {
    STOW,
    TO_INTAKE_STAGE,
    INTAKE_STAGE,
    INTAKE,
    MANUAL_SCORE,
    MANUAL_SHELF,
    AUTO_SCORE,
    AUTO_SHELF,
    FLOOR_PICKUP,
    RELEASE_GAME_PIECE,
    SHELF_WAIT
  }
}
