package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AutoSwitch;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ToggleVirtualSwitchCommand extends InstantCommand {
  private AutoSwitch autoSwitch;
  private static Logger logger;

  public ToggleVirtualSwitchCommand(AutoSwitch autoSwitch) {
    this.autoSwitch = autoSwitch;
    logger = LoggerFactory.getLogger(ToggleVirtualSwitchCommand.class);
  }

  @Override
  public void initialize() {
    autoSwitch.toggleVirtualSwitch();
    logger.info("toggledSwitch:command");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
