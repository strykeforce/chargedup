package frc.robot.subsystems;

import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  private TargetLevel targetLevel;
  private TargetCol targetCol;
  private GamePiece gamePiece;
  private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);
  private boolean autoStaging;

  public RobotStateSubsystem(
      TargetLevel targetLevel, TargetCol targetCol, GamePiece gamePiece, boolean autoStaging) {
    this.targetLevel = targetLevel;
    this.targetCol = targetCol;
    this.gamePiece = gamePiece;
    this.autoStaging = autoStaging;
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
}
