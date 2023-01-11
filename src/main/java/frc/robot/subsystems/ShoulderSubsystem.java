package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    private TalonFX shoulderTalon;
    private DigitalInput zeroShoulderInput = new DigitalInput(Constants.ShoulderConstants.kZeroId);

    private final Logger logger = LoggerFactory.getLogger(this.getClass());

    public ShoulderSubsystem() {
        shoulderTalon = new TalonFX(Constants.ShoulderConstants.kShoulderId);
        shoulderTalon.configAllSettings(Constants.ShoulderConstants.getShoulderTalonConfig());
    }

    public void setPos(double location) {
        shoulderTalon.set(ControlMode.Position, location);
    }

    public void setPct(double pct) {
        shoulderTalon.set(ControlMode.PercentOutput, pct);
    }

    public void zeroShoulder() {
        if (zeroShoulderInput.get()) {
            double absolute = shoulderTalon.getSensorCollection().getIntegratedSensorAbsolutePosition();
            double offset = absolute - Constants.ShoulderConstants.kShoulderZeroTicks;

            shoulderTalon.setSelectedSensorPosition(offset);

            logger.info(
                    "Shoulder zeroed; offset: {} zeroTicks: {} absolute: {}",
                    offset,
                    Constants.ShoulderConstants.kShoulderZeroTicks,
                    absolute);

        } else {
            shoulderTalon.configPeakOutputForward(0, 0);
            shoulderTalon.configPeakOutputReverse(0, 0);
            logger.error("Shoulder zero failed. Killing shoulder...");
        }
    }
}
