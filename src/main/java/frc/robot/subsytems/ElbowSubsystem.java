package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;

public class ElbowSubsystem extends SubsystemBase {
    private TalonFX leftMainFalcon;
    private TalonFX rightFollowFalcon;

    public ElbowSubsystem() {
        leftMainFalcon = new TalonFX(ElbowConstants.kLeftMainFalconID);
        leftMainFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());
        rightFollowFalcon = new TalonFX(ElbowConstants.kRightFollowFalconID);
        rightFollowFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());
        rightFollowFalcon.follow(leftMainFalcon);
    }
}
