package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final double kCanConfigTimeout = 10; // ms

    public static final class ShoulderConstants {
        public static final int kShoulderId = 0; // FIXME
        public static final int kZeroId = 0; // FIXME

        public static final double kShoulderZeroTicks = 0; // FIXME

        public static final double kMaxFwd = 0; // FIXME
        public static final double kMaxRev = 0; // FIXME

        public static TalonFXConfiguration getShoulderTalonConfig() {
            TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

            shoulderConfig.supplyCurrLimit.currentLimit = 40;
            shoulderConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
            shoulderConfig.supplyCurrLimit.triggerThresholdTime = .04;
            shoulderConfig.supplyCurrLimit.enable = true;

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
    }
}
