package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.RobotStateConstants;
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
  private Alliance allianceColor = DriverStation.getAlliance();
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

  public void setAllianceColor(Alliance alliance) {
    this.allianceColor = alliance;
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public Pose2d getShelfPosAutoDrive(TargetCol tempTargetCol, boolean isBlue) {
    int multiplier = 0;
    if (tempTargetCol.equals(TargetCol.LEFT)) multiplier = 1;
    if (tempTargetCol.equals(TargetCol.RIGHT)) multiplier = -1;
    if (isBlue) multiplier *= -1;

    if (isBlue)
      return new Pose2d(
          new Translation2d(
              RobotStateConstants.kShelfBlue.getX() + RobotStateConstants.kShelfOffset * multiplier,
              RobotStateConstants.kShelfBlue.getY()),
          new Rotation2d(Math.PI));
    return new Pose2d(
        new Translation2d(
            RobotStateConstants.kShelfRed.getX() + RobotStateConstants.kShelfOffset * multiplier,
            RobotStateConstants.kShelfRed.getY()),
        new Rotation2d(0.0));
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
      rotation = 0;
    }
    int multiplier = 0;
    if (targetCol.equals(TargetCol.LEFT)) multiplier = 1;
    if (targetCol.equals(TargetCol.RIGHT)) multiplier = -1;
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
}
