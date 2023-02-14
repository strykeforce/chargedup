package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  private IntakeSubsystem intakeSubsystem;
  // private ArmSubsystem armSubsystem;
  private TargetLevel targetLevel = TargetLevel.NONE;
  private TargetCol targetCol = TargetCol.NONE;
  private GamePiece gamePiece = GamePiece.NONE;
  private RobotState currRobotState;
  private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);
  private Alliance allianceColor = DriverStation.getAlliance();
  private boolean isAutoStage;

  public RobotStateSubsystem(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
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
    return autoStaging;
  }

  public void setAutoStaging(boolean enable) {
    this.autoStaging = enable;
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

  public void maunalStage() {
    // if we do have a game piece
    isAutoStage  = false;
    if (targetLevel == TargetLevel.HIGH) {
      currRobotState = RobotState.LVL_3_SCORE;
    } else if (targetLevel == TargetLevel.MID) {
      currRobotState = RobotState.LVL_2_SCORE;
    } else if (targetLevel == TargetLevel.LOW) {
      currRobotState = RobotState.LVL_1_SCORE;
    }
  }

  public void autoStage() {
    // if we do have a game piece
    isAutoStage = true;
    if (targetLevel == TargetLevel.HIGH) {
      currRobotState = RobotState.LVL_3_SCORE;
    } else if (targetLevel == TargetLevel.MID) {
      currRobotState = RobotState.LVL_2_SCORE;
    } else if (targetLevel == TargetLevel.LOW) {
      currRobotState = RobotState.LVL_1_SCORE;
    }
  }

  @Override
  public void periodic() {
    switch (currRobotState) {
      case STOW:
        intakeSubsystem.retractIntake();
        if (intakeSubsystem.isIntakeAtPos()) {
          // armSubsystem.setArmState(ArmState.STOW);
        }
        break;
      case SHELF_PICKUP:
        break;
      case FLOOR_PICKUP:
        break;
      case LVL_1_SCORE:
        if (isAutoStage) {
          // goto position
        }
        // armsubsystem: goto lvl 1 position
        intakeSubsystem.retractIntake();
        break;
      case LVL_2_SCORE:
        break;
      case LVL_3_SCORE:
        break;
      case INTAKE:
        intakeSubsystem.startIntaking();
        break;
      case AUTO_BALANCE:
        break;
      case AUTO_ALIGN:
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
    SHELF_PICKUP,
    FLOOR_PICKUP,
    LVL_1_SCORE,
    LVL_2_SCORE,
    LVL_3_SCORE,
    INTAKE,
    AUTO_BALANCE,
    AUTO_ALIGN
  }
}
