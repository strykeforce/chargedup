package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RGBlightsSubsystem extends MeasurableSubsystem {
  private PWM red_channel;
  private PWM green_channel;
  private PWM blue_channel;
  private Logger logger = LoggerFactory.getLogger(RGBlightsSubsystem.class);

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public RGBlightsSubsystem() {
    red_channel = new PWM(0);
    green_channel = new PWM(1);
    blue_channel = new PWM(2);
    red_channel.setPeriodMultiplier(PeriodMultiplier.k1X);
    green_channel.setPeriodMultiplier(PeriodMultiplier.k1X);
    blue_channel.setPeriodMultiplier(PeriodMultiplier.k1X);
  }

  public void setColor(double red, double green, double blue) {
    double R = red * 5000 - 500;
    if (R < 0) {
      R = 0;
    } else if (R > 4000) {
      R = 4000;
    }

    double G = green * 5000 - 500;
    if (G < 0) {
      G = 0;
    } else if (G > 4000) {
      G = 4000;
    }

    double B = blue * 5000 - 500;
    if (B < 0) {
      B = 0;
    } else if (B > 4000) {
      B = 4000;
    }

    red_channel.setRaw((int) R);
    green_channel.setRaw((int) G);
    blue_channel.setRaw((int) B);
    logger.info("set color to R {} G {} B {}", R, G, B);
  }

  public void setCubeColor() {
    this.setColor(0.43, 0.01, 0.43);
  }

  public void setConeColor() {
    this.setColor(0.78, 0.66, 0.06);
  }

  public void setOff() {
    this.setColor(0, 0, 0);
  }
}
