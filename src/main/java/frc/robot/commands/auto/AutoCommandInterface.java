package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;

public interface AutoCommandInterface extends Command {
  public void generateTrajectory();

  public boolean hasGenerated();
}
