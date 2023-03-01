package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SetAllianceCommand extends InstantCommand {
  public final Alliance alliance;
  public final RobotContainer robotContainer;
  private static final Logger logger = LoggerFactory.getLogger(SetAllianceCommand.class);

  public SetAllianceCommand(Alliance alliance, RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  @Override
  public void initialize() {
    robotContainer.setAllianceColor(alliance);
    logger.info("Manual Alliance Color: {}", alliance);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
