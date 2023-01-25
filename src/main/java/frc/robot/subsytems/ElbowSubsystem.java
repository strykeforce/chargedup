package frc.robot.subsytems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ElbowConstants;
import java.util.Set;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.CanifierMeasurable;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ElbowSubsystem extends MeasurableSubsystem {
  private TalonFX elbowFalcon;
  private int setPointTicks = 0;
  private CANifier remoteEncoder;

  public ElbowSubsystem() {
    elbowFalcon = new TalonFX(ElbowConstants.kElbowFalconID);
    elbowFalcon.configAllSettings(ElbowConstants.getElbowFalonConfig());

    remoteEncoder = new CANifier(ElbowConstants.kRemoteEncoderID);

    elbowFalcon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    elbowFalcon.configRemoteFeedbackFilter(
        ElbowConstants.kRemoteEncoderID, RemoteSensorSource.CANifier_Quadrature, 0);

    zeroElbow();

    Rotation2d bob = new Rotation2d(5);
  }

  private double getPulseWidthFor(PWMChannel channel) {
    double[] pulseWidthandPeriod = new double[2];
    remoteEncoder.getPWMInput(channel, pulseWidthandPeriod);
    return pulseWidthandPeriod[0];
  }

  private void zeroElbow() {
    double offset = getPulseWidthFor(PWMChannel.PWMChannel0) - ElbowConstants.kZeroTicks;
    elbowFalcon.setSelectedSensorPosition(offset);
  }

  public void rotateOpenLoop(double percentOutput) {
    elbowFalcon.set(ControlMode.PercentOutput, percentOutput);
  }

  public void rotateClosedLoop(int posTicks) {
    elbowFalcon.set(ControlMode.MotionMagic, posTicks);
    setPointTicks = posTicks;
  }

  public boolean isElbowAtPos() {
    return Math.abs(setPointTicks - elbowFalcon.getSelectedSensorPosition())
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
    telemetryService.register(elbowFalcon);
    telemetryService.register(new CanifierMeasurable(remoteEncoder));
  }
}
