package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;

public class ElbowSubsystem extends SubsystemBase {
    private TalonFX leftMainFalcon;
    private TalonFX rightFollowFalcon;
    private int targetPosTicks = 0;

    public ElbowSubsystem() {
        leftMainFalcon = new TalonFX(ElbowConstants.kLeftMainFalconID);
        leftMainFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());
        rightFollowFalcon = new TalonFX(ElbowConstants.kRightFollowFalconID);
        rightFollowFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());
        rightFollowFalcon.follow(leftMainFalcon);
    }

    public void rotateClosedLoop(int posTicks) {
        leftMainFalcon.set(ControlMode.MotionMagic, posTicks);
        targetPosTicks = posTicks;
    }
}
