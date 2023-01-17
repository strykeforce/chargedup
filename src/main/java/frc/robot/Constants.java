package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
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
  public static final double kCanConfigTimeout = 10; // ms

  public static final class ShoulderConstants {
    public static final int kShoulderId = 30; // FIXME
    public static final int kZeroId = 0; // FIXME

    public static final double kShoulderZeroTicks = 0; // FIXME

    public static final double kMaxFwd = 0; // FIXME
    public static final double kMaxRev = 0; // FIXME

    public static final double kZeroRads = 0; // FIXME

    public static final double kTicksPerDeg = 0; // FIXME

    public static final double kAllowedError = 0; // FIXME

    public static TalonSRXConfiguration getShoulderTalonConfig() {
      TalonSRXConfiguration shoulderConfig = new TalonSRXConfiguration();

      // shoulderConfig.slot0.kP = 0.0;
      // shoulderConfig.slot0.kI = 0.0;
      // shoulderConfig.slot0.kD = 0.0;
      // shoulderConfig.slot0.kF = 0.0;
      // shoulderConfig.slot0.integralZone = 0;
      // shoulderConfig.slot0.maxIntegralAccumulator = 0;
      // shoulderConfig.slot0.allowableClosedloopError = 0;
      // shoulderConfig.motionCruiseVelocity = 0;
      // shoulderConfig.motionAcceleration = 0;

      shoulderConfig.forwardSoftLimitEnable = true;
      shoulderConfig.forwardSoftLimitThreshold = kMaxFwd;
      shoulderConfig.reverseSoftLimitEnable = true;
      shoulderConfig.reverseSoftLimitThreshold = kMaxRev;
      shoulderConfig.neutralDeadband = 0.01;
      shoulderConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      shoulderConfig.velocityMeasurementWindow = 64;
      shoulderConfig.voltageCompSaturation = 12;
      shoulderConfig.voltageMeasurementFilter = 32;

      return shoulderConfig;
    }

    public static SupplyCurrentLimitConfiguration getShoulderTalonSupplyLimitConfig() {
      SupplyCurrentLimitConfiguration shoulderSupplyConfig = new SupplyCurrentLimitConfiguration();

      shoulderSupplyConfig.currentLimit = 40;
      shoulderSupplyConfig.triggerThresholdCurrent = 45;
      shoulderSupplyConfig.triggerThresholdTime = .04;
      shoulderSupplyConfig.enable = true;

      return shoulderSupplyConfig;
    }
  }
}
