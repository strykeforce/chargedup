package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForMatchTimeCommand extends CommandBase {
  private double matchTime;

  public WaitForMatchTimeCommand(double matchTime) {
    this.matchTime = matchTime;
  }

  @Override
  public boolean isFinished() {
    return DriverStation.getMatchTime() <= matchTime;
  }
}
