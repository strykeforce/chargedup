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
  private ArmSubsystem armSubsystem;
  private HandSubsystem handSubsystem;
  private TargetLevel targetLevel = TargetLevel.NONE;
  private TargetCol targetCol = TargetCol.NONE;
  private GamePiece gamePiece = GamePiece.NONE;
  private RobotState currRobotState = RobotState.STOW;
  private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);
  private Alliance allianceColor = DriverStation.getAlliance();
  private boolean isAutoStage;

  public RobotStateSubsystem(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, HandSubsystem handSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.handSubsystem = handSubsystem;
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

  public void maunalStage() {
    
  }

  public void autoStage() {
    
  }

  @Override
  public void periodic() {
    switch (currRobotState) {
      case STOW:
        // intake off
        // close hand
        // pull arm in
        // retract intake
        break;
      case TO_INTAKE_STAGE:
        // (from) STOW
        // extend intake
        // turn on intake
        // elbow to position
        // open hand
        // (to) INTAKE_STAGE

        intakeSubsystem.startIntaking();
        if (!intakeSubsystem.isIntakeAtPos()) break;

        armSubsystem.toIntakeStagePos();
        if (!armSubsystem.isFinished()) break;

        handSubsystem.s
        // if (!armSubsystem.isFinished()) break;
      case INTAKE_STAGE:
        // (from) TO_INTAKE_STAGE
        // beam break
        // (to) INTAKE

        if (!intakeSubsystem.getIsBeamBreakActive()) break;
      case INTAKE:
        // (from) INTAKE_STAGE
        // intake off
        // intake in
        // elevator down
        // hand close
        // small delay
        // set game piece cube
        // (to) STOW

        intakeSubsystem.retractIntake();
        if (!intakeSubsystem.isIntakeAtPos()) break;

        armSubsystem.toIntakePos();
        if (!armSubsystem.isFinished()) break;

        // close hand

        // delay

        setGamePiece(GamePiece.CUBE);

        currRobotState = RobotState.STOW;

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

  private enum RobotState {
    STOW,
    TO_INTAKE_STAGE,
    INTAKE_STAGE,
    INTAKE,
  }
}
