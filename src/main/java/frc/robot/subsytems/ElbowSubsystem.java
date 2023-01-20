package frc.robot.subsytems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;

public class ElbowSubsystem extends SubsystemBase {
    private TalonFX leftMainFalcon;
    private TalonFX rightFollowFalcon;
    private int setPointTicks = 0;
    private CANifier remoteEncoder;

    public ElbowSubsystem() {
        leftMainFalcon = new TalonFX(ElbowConstants.kLeftMainFalconID);
        leftMainFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());
        rightFollowFalcon = new TalonFX(ElbowConstants.kRightFollowFalconID);
        rightFollowFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());
        rightFollowFalcon.follow(leftMainFalcon);

        remoteEncoder = new CANifier(ElbowConstants.kRemoteEncoderID);

        leftMainFalcon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        leftMainFalcon.configRemoteFeedbackFilter(ElbowConstants.kRemoteEncoderID, RemoteSensorSource.CANifier_Quadrature, 0);

        zeroElbow();
    }

    private void zeroElbow() {

    }

    public void rotateOpenLoop(double percentOutput) {
        leftMainFalcon.set(ControlMode.PercentOutput, percentOutput);
    }

    public void rotateClosedLoop(int posTicks) {
        leftMainFalcon.set(ControlMode.MotionMagic, posTicks);
        setPointTicks = posTicks;
    }

    public boolean isElbowAtPos() {
        return Math.abs(setPointTicks - leftMainFalcon.getSelectedSensorPosition()) < ElbowConstants.kCloseEnoughTicks;
    }
}
