package frc.robot.subsystems;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  private TargetLevel targetLevel;
  private TargetCol targetCol;
  private GamePiece gamePiece;  

  public RobotStateSubsystem(TargetLevel targetLevel, TargetCol targetCol, GamePiece gamePiece) {
    this.targetLevel = targetLevel;
    this.targetCol = targetCol;
    this.gamePiece = gamePiece;
  }

  public void setTargetLevel(TargetLevel targetLevel) {
    this.targetLevel = targetLevel;
  }

  public TargetLevel getTargetLevel() {
    return targetLevel;
  }

  public void setTargetCol(TargetCol targetCol) {
    this.targetCol = targetCol;
  }

  public TargetCol getTargetCol() {
    return targetCol;
  }

  public void setGamePiece(GamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }

  public GamePiece getGamePiece() {
    return gamePiece;
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
