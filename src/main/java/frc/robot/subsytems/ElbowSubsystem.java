package frc.robot.subsytems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.ElbowConstants;
import java.util.Set;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.CanifierMeasurable;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElbowSubsystem extends MeasurableSubsystem {
  private TalonFX leftMainFalcon;
  private TalonFX rightFollowFalcon;
  private int setPointTicks = 0;
  private CANifier remoteEncoder;

  public ElbowSubsystem() {
    leftMainFalcon = new TalonFX(ElbowConstants.kLeftMainFalconID);
    leftMainFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());
    rightFollowFalcon = new TalonFX(ElbowConstants.kRightFollowFalconID);
    rightFollowFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());
    rightFollowFalcon.follow(leftMainFalcon, FollowerType.AuxOutput1);

    remoteEncoder = new CANifier(ElbowConstants.kRemoteEncoderID);

    leftMainFalcon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    leftMainFalcon.configRemoteFeedbackFilter(
        ElbowConstants.kRemoteEncoderID, RemoteSensorSource.CANifier_Quadrature, 0);

    zeroElbow();
  }

  private double getPulseWidthFor(PWMChannel channel) {
    double[] pulseWidthandPeriod = new double[2];
    remoteEncoder.getPWMInput(channel, pulseWidthandPeriod);
    return pulseWidthandPeriod[0];
  }

  private void zeroElbow() {
    double offset = getPulseWidthFor(PWMChannel.PWMChannel0) - ElbowConstants.kZeroTicks;
    leftMainFalcon.setSelectedSensorPosition(offset);
    rightFollowFalcon.setSelectedSensorPosition(offset);
  }

  public void rotateOpenLoop(double percentOutput) {
    leftMainFalcon.set(ControlMode.PercentOutput, percentOutput);
  }

  public void rotateClosedLoop(int posTicks) {
    leftMainFalcon.set(ControlMode.MotionMagic, posTicks);
    setPointTicks = posTicks;
  }

  public boolean isElbowAtPos() {
    return Math.abs(setPointTicks - leftMainFalcon.getSelectedSensorPosition())
        < ElbowConstants.kCloseEnoughTicks;
  }

  @Override
  public void periodic() {
      // nothing here yet :)
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    telemetryService.register(leftMainFalcon);
    telemetryService.register(new CanifierMeasurable(remoteEncoder));
  }
}
