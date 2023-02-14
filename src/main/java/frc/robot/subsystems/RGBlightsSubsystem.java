package frc.robot.subsystems;

import edu.wpi.first.hal.PWMConfigDataResult;
import edu.wpi.first.wpilibj.PWM;
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
    PWMConfigDataResult temp = red_channel.getRawBounds();
    logger.info(
        "max:{} dbmax:{} center:{} dbmin:{} min:{}",
        temp.max,
        temp.deadbandMax,
        temp.center,
        temp.deadbandMin,
        temp.min);
    logger.info("{}", red_channel.getRaw());
  }

  public void setColor(int red, int green, int blue) {
    red_channel.setRaw(red);
    green_channel.setRaw(green);
    blue_channel.setRaw(blue);
    logger.info("set color to R {} G {} B {}", red, green, blue);
    logger.info(
        "red:{} green:{} blue:{}",
        red_channel.getRaw(),
        green_channel.getRaw(),
        blue_channel.getRaw());
  }

  public void setCubeColor() {
    this.setColor(111, 3, 111);
  }

  public void setConeColor() {
    this.setColor(500, 169, 16);
  }
}
