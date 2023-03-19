package frc.robot.subsystems;

public interface ArmComponent {
  public void setSoftLimits(double minTicks, double maxTicks);

  public double getPos();

  public boolean canStartNextAxis(double canStartTicks);
}
