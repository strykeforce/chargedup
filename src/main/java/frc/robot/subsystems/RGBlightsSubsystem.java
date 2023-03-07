package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
// import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RGBlightsSubsystem extends MeasurableSubsystem {
  // private PWM red_channel;
  // private PWM green_channel;
  // private PWM blue_channel;
  private DigitalOutput red_channel_D;
  private DigitalOutput green_channel_D;
  private DigitalOutput blue_channel_D;

  private Logger logger = LoggerFactory.getLogger(RGBlightsSubsystem.class);
  private double redSetpoint = 1.0;
  private double greenSetpoint = 0.0;
  private double blueSetpoint = 0.0;

  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public RGBlightsSubsystem() {
    // red_channel = new PWM(0);
    // green_channel = new PWM(1);
    // blue_channel = new PWM(2);
    // red_channel.setPeriodMultiplier(PeriodMultiplier.k1X);
    // green_channel.setPeriodMultiplier(PeriodMultiplier.k1X);
    // blue_channel.setPeriodMultiplier(PeriodMultiplier.k1X);
    red_channel_D = new DigitalOutput(7);
    red_channel_D.setPWMRate(2000);
    red_channel_D.enablePWM(0);
    green_channel_D = new DigitalOutput(8);
    green_channel_D.setPWMRate(2000);
    green_channel_D.enablePWM(0);
    blue_channel_D = new DigitalOutput(9);
    blue_channel_D.setPWMRate(2000);
    blue_channel_D.enablePWM(0);
    setColor(1.0, 0.0, 0.0);
  }

  public void setColor(Double red, Double green, Double blue) {
    // logger.info("Set RGB color to: {}, {}, {}", red, green, blue);
    // double R = red * 5000 - 500;
    // if (R < 0) {
    //   R = 0;
    // } else if (R > 4000) {
    //   R = 4000;
    // }

    // double G = green * 5000 - 500;
    // if (G < 0) {
    //   G = 0;
    // } else if (G > 4000) {
    //   G = 4000;
    // }

    // double B = blue * 5000 - 500;
    // if (B < 0) {
    //   B = 0;
    // } else if (B > 4000) {
    //   B = 4000;
    // }

    // red_channel.setRaw((int) R);
    // green_channel.setRaw((int) G);
    // blue_channel.setRaw((int) B);

    // logger.info("set color to R {} G {} B {}", R, G, B);
    red_channel_D.updateDutyCycle(red);
    green_channel_D.updateDutyCycle(green);
    blue_channel_D.updateDutyCycle(blue);
    if(red != redSetpoint || blue != blueSetpoint || green != greenSetpoint){
      logger.info("Lights Changed to: ({}, {}, {})", red, green, blue);
      redSetpoint = red;
      greenSetpoint = green;
      blueSetpoint = blue;
    }
  }

  public void setCubeColor() {
    this.setColor(0.43, 0.01, 0.43);
  }

  public void setConeColor() {
    this.setColor(0.78, 0.66, 0.06);
  }

  public void setOff() {
    this.setColor(0.0, 0.0, 0.0);
  }
}
