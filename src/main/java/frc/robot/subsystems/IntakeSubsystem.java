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
    private double lastAbsPos = 0;
    private int zeroStableCounts = 0;
    private boolean isIntakeExtended = false;
    private boolean didZero = false;
    private final Logger logger = LoggerFactory.getLogger(IntakeSubsystem.class);

    public IntakeSubsystem() {
        intakeFalcon = new TalonFX(IntakeConstants.kIntakeFalconID);
        rollerFalcon = new TalonFX(IntakeConstants.kRollerFalconID);
        intakeExtendFalcon = new TalonFX(IntakeConstants.kIntakeExtendFalconID);

        zeroIntake();
    }

    public void setIntakeSpeed(double speed) {
        intakeFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void setRollerSpeed(double speed) {
        rollerFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void extendClosedLoop() {
        intakeExtendFalcon.set(ControlMode.MotionMagic, IntakeConstants.kExtendPosTicks);
        intakeSetPointTicks = IntakeConstants.kExtendPosTicks;
        isIntakeExtended = true;
        logger.info("Intake is extending to {}", IntakeConstants.kExtendPosTicks);
        setIntakeSpeed(1);
        setRollerSpeed(1);
    }

    public void retractClosedLoop() {
        intakeExtendFalcon.set(ControlMode.MotionMagic, IntakeConstants.kRetractPosTicks);
        intakeSetPointTicks = IntakeConstants.kRetractPosTicks;
        isIntakeExtended = false;
        logger.info("Intake is retracting to {}", IntakeConstants.kRetractPosTicks);
        setIntakeSpeed(0);
        setRollerSpeed(0);
    }

    public boolean isIntakeAtPos() {
        return Math.abs(intakeSetPointTicks - intakeExtendFalcon.getSelectedSensorPosition()) < IntakeConstants.kCloseEnoughTicks;
    }

    public boolean getIsIntakeExtended() {
        return isIntakeExtended;
    }

    public void zeroIntake() {
        double absPos = intakeExtendFalcon.getSensorCollection().getIntegratedSensorAbsolutePosition();
        if (Math.abs(absPos - lastAbsPos) <= IntakeConstants.kZeroStableBand) zeroStableCounts++;
        else zeroStableCounts = 0;
        lastAbsPos = absPos;
    
        if (zeroStableCounts > IntakeConstants.kZeroStableCounts) {
          double offset = absPos - IntakeConstants.kIntakeZeroTicks;
          intakeExtendFalcon.setSelectedSensorPosition(offset);
          didZero = true;
          logger.info(
              "Intake zeroed; offset: {} zeroTicks: {} absPosition: {}",
              offset,
              IntakeConstants.kIntakeZeroTicks,
              absPos);
          retractClosedLoop();
        }
    }

    @Override
    public void periodic() {
      if (!didZero) zeroIntake();
    }
}
