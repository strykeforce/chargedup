package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intakeFalcon;
    private TalonFX rollerFalcon;
    private TalonFX intakeExtendFalcon;
    private double intakeSetPointTicks;
    private boolean isIntakeExtended = false;
    private final Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);

    public IntakeSubsystem() {
        intakeFalcon = new TalonFX(IntakeConstants.kIntakeFalconID);
        rollerFalcon = new TalonFX(IntakeConstants.kRollerFalconID);
        intakeExtendFalcon = new TalonFX(IntakeConstants.kIntakeExtendFalconID);
    }

    public void setIntakeSpeed(double speed) {
        intakeFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void setRollerSpeed(double speed) {
        rollerFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void extendClosedLoop() {
        intakeExtendFalcon.set(ControlMode.MotionMagic, IntakeConstants.kIntakeExtendPosTicks);
        intakeSetPointTicks = IntakeConstants.kIntakeExtendPosTicks;
        isIntakeExtended = true;
        logger.info("Intake is extending to {}", IntakeConstants.kIntakeExtendPosTicks);
    }

    public void retractClosedLoop() {
        intakeExtendFalcon.set(ControlMode.MotionMagic, IntakeConstants.kIntakeRetractPosTicks);
        intakeSetPointTicks = IntakeConstants.kIntakeRetractPosTicks;
        isIntakeExtended = false;
        logger.info("Intake is retracting to {}", IntakeConstants.kIntakeRetractPosTicks);
    }

    public boolean isIntakeAtPos() {
        return Math.abs(intakeSetPointTicks - intakeExtendFalcon.getSelectedSensorPosition()) < IntakeConstants.kCloseEnoughTicks;
    }

    public boolean getIsIntakeExtended() {
        return isIntakeExtended;
    }
}
