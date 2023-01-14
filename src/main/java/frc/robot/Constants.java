package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
  public static class ElevatorConstants {
    public static final int kLeftMainId = 0; // FIXME
    public static final int kRightFollowerId = 0; // FIXME

    public static TalonFXConfiguration getElevatorFalconConfig() {
      TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

      elevatorConfig.supplyCurrLimit.currentLimit = 80;
      elevatorConfig.supplyCurrLimit.triggerThresholdCurrent = 90;
      elevatorConfig.supplyCurrLimit.triggerThresholdTime = 0.1;
      elevatorConfig.supplyCurrLimit.enable = true;

      elevatorConfig.statorCurrLimit.currentLimit = 100.0;
      elevatorConfig.statorCurrLimit.triggerThresholdCurrent = 120.0;
      elevatorConfig.statorCurrLimit.triggerThresholdTime = 0.1;
      elevatorConfig.statorCurrLimit.enable = true;

      elevatorConfig.slot0.kP = 1.0;
      elevatorConfig.slot0.kI = 0.0;
      elevatorConfig.slot0.kD = 0.0;
      elevatorConfig.slot0.kF = 0.065;
      elevatorConfig.slot0.integralZone = 0;
      elevatorConfig.slot0.maxIntegralAccumulator = 0;
      elevatorConfig.slot0.allowableClosedloopError = 0;
      elevatorConfig.motionCruiseVelocity = 5_000;
      elevatorConfig.motionAcceleration = 30_000;
      elevatorConfig.forwardSoftLimitEnable = false;
      elevatorConfig.forwardSoftLimitThreshold = 0;
      elevatorConfig.reverseSoftLimitEnable = false;
      elevatorConfig.reverseSoftLimitThreshold = -200_000;
      elevatorConfig.neutralDeadband = 0.01;
      elevatorConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      elevatorConfig.velocityMeasurementWindow = 64;
      elevatorConfig.voltageCompSaturation = 12;
      elevatorConfig.voltageMeasurementFilter = 32;

      return elevatorConfig;
    }

    public static SupplyCurrentLimitConfiguration getElevatorSupplyLimitConfig() {
      SupplyCurrentLimitConfiguration elevatorSupplyConfig = new SupplyCurrentLimitConfiguration();

      elevatorSupplyConfig.currentLimit = 40;
      elevatorSupplyConfig.triggerThresholdCurrent = 45;
      elevatorSupplyConfig.triggerThresholdTime = .04;
      elevatorSupplyConfig.enable = true;

      return elevatorSupplyConfig;
    }

    public static SupplyCurrentLimitConfiguration getElevatorZeroSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 100, 120, 1.0);
    }
  }
}
