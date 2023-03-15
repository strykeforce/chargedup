package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoWaitForMatchTimeCommand extends CommandBase {
  private double waitUntil;

  public AutoWaitForMatchTimeCommand(double waitUntil) {
    this.waitUntil = waitUntil;
  }

  @Override
  public boolean isFinished() {
    return DriverStation.getMatchTime() <= waitUntil;
  }
}
