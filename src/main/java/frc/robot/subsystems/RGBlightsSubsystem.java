package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RGBlightsSubsystem extends MeasurableSubsystem {
  private PWM red_channel;
  private PWM green_channel;
  private PWM blue_channel;

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public RGBlightsSubsystem() {
    red_channel = new PWM(0);
    green_channel = new PWM(1);
    blue_channel = new PWM(2);
  }

  public void setColor(int red, int green, int blue) {
    red_channel.setRaw(red);
    green_channel.setRaw(green);
    blue_channel.setRaw(blue);
  }

  public void setCubeColor() {
    this.setColor(111, 3, 111);
  }

  public void setConeColor() {
    this.setColor(199, 169, 16);
  }
}
