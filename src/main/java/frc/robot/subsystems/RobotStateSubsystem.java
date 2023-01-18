package frc.robot.subsystems;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  private TargetLevel targetLevel;
  private GamePiece gamePiece;

  public RobotStateSubsystem(TargetLevel targetLevel, GamePiece gamePiece) {
    this.targetLevel = targetLevel;
    this.gamePiece = gamePiece;
  }

  public void setTargetLevel(TargetLevel targetLevel) {
    this.targetLevel = targetLevel;
  }

  public TargetLevel getTargetLevel() {
    return targetLevel;
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
  public enum GamePiece {
    NONE,
    CUBE,
    CONE  
  };
}
