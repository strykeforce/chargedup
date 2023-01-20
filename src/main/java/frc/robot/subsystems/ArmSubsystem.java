package frc.robot.subsystems;

import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ArmSubsystem extends MeasurableSubsystem {
  private Logger logger = LoggerFactory.getLogger(ArmSubsystem.class);
  private ArmState armState;
  private ShoulderSubsystem shoulderSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ElbowSubsystem elbowSubsystem;

  public ArmSubsystem(
      ShoulderSubsystem shoulderSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElbowSubsystem elbowSubsystem) {
    this.armState = ArmState.STOWED;
    this.shoulderSubsystem = shoulderSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elbowSubsystem = elbowSubsystem;
  }

  public void setArmState(ArmState newArmState) {
    this.armState = newArmState;
    shoulderSubsystem.setPos(armState.shoulderPos);
    elevatorSubsystem.setPos(armState.elevatorPos);
    elbowSubsystem.rotateClosedLoop(armState.elbowPos);

    logger.info("Setting armState to {}", newArmState.name());
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  @Override
  public void periodic() {
    switch (armState) {
      case STOWED:
        break;
      case LOW:
        break;
      case MID_CUBE:
        break;
      case HIGH_CUBE:
        break;
      case MID_CONE:
        break;
      case HIGH_CONE:
        break;
      case PORTAL:
        break;
      default:
        break;
    }
  }

  public enum ArmState {
    STOWED(0, 0, 0),
    LOW(0, 0, 0),
    MID_CUBE(0, 0, 0),
    HIGH_CUBE(0, 0, 0),
    MID_CONE(0, 0, 0),
    HIGH_CONE(0, 0, 0),
    PORTAL(0, 0, 0);

    public final double shoulderPos;
    public final double elevatorPos;
    public final double elbowPos;

    ArmState(double shoulderPos, double elevatorPos, double elbowPos) {
      this.shoulderPos = shoulderPos;
      this.elevatorPos = elevatorPos;
      this.elbowPos = elbowPos;
    }
  }
}
