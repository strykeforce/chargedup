package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import java.util.Set;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElevatorSubsystem extends MeasurableSubsystem {
  private TalonFX leftMain, rightFollower;

  public ElevatorSubsystem() {
    leftMain = new TalonFX(Constants.ElevatorConstants.kLeftMainId);
    rightFollower = new TalonFX(Constants.ElevatorConstants.kRightFollowerId);

    leftMain.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    leftMain.configSupplyCurrentLimit(Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    leftMain.setInverted(TalonFXInvertType.Clockwise);
    
    rightFollower.configAllSettings(Constants.ElevatorConstants.getElevatorFalconConfig());
    rightFollower.configSupplyCurrentLimit(
        Constants.ElevatorConstants.getElevatorSupplyLimitConfig());
    rightFollower.setInverted(TalonFXInvertType.CounterClockwise);
    rightFollower.follow(leftMain, FollowerType.PercentOutput);
  }

  public void setPos(double location) {
    leftMain.set(TalonFXControlMode.MotionProfile, location);
  }

  public void setPct(double pct) {
    leftMain.set(TalonFXControlMode.PercentOutput, pct);
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(leftMain);
    telemetryService.register(rightFollower);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }
}
