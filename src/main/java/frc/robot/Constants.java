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
  public static final int kTalonConfigTimeout = 10; // ms

  public static class HandConstants {
    public static int kHandTalonId = 40;
    public static int kWristTalonId = 0; // FIXME

    public static final double kMaxFwd = 0; // FIXME
    public static final double kMaxRev = 0; // FIXME

    public static final double kHandZeroSpeed = 0.1;
    public static final double kZeroTargetSpeedTicksPer100ms = 5;
    public static final int kZeroStableCounts = 25;

    public static final double kAllowedError = 0; // FIXME

    public static final double kCubeGrabbingPositionLeft = 0; // FIXME
    public static final double kConeGrabbingPositionLeft = 0; // FIXME
    public static final double kCubeGrabbingPositionRight = 0; // FIXME
    public static final double kConeGrabbingPositionRight = 0; // FIXME

    public static TalonSRXConfiguration getHandTalonConfig() {
      TalonSRXConfiguration handConfig = new TalonSRXConfiguration();

      // shoulderConfig.slot0.kP = 0.0;
      // shoulderConfig.slot0.kI = 0.0;
      // shoulderConfig.slot0.kD = 0.0;
      // shoulderConfig.slot0.kF = 0.0;
      // shoulderConfig.slot0.integralZone = 0;
      // shoulderConfig.slot0.maxIntegralAccumulator = 0;
      // shoulderConfig.slot0.allowableClosedloopError = 0;
      // shoulderConfig.motionCruiseVelocity = 0;
      // shoulderConfig.motionAcceleration = 0;

      handConfig.forwardSoftLimitEnable = true;
      handConfig.forwardSoftLimitThreshold = kMaxFwd;
      handConfig.reverseSoftLimitEnable = true;
      handConfig.reverseSoftLimitThreshold = kMaxRev;
      handConfig.neutralDeadband = 0.01;
      handConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      handConfig.velocityMeasurementWindow = 64;
      handConfig.voltageCompSaturation = 12;
      handConfig.voltageMeasurementFilter = 32;

      return handConfig;
    }

    public static SupplyCurrentLimitConfiguration getHandSupplyLimitConfig() {
      return new SupplyCurrentLimitConfiguration(true, 40, 45, .04);
    }

    public static SupplyCurrentLimitConfiguration getHandZeroSupplyCurrentLimit() {
      return new SupplyCurrentLimitConfiguration(true, 5, 5, 0.1);
    }
  }
}
