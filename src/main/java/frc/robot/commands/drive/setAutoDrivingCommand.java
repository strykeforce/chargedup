package frc.robot.commands.drive;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class setAutoDrivingCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private boolean isAutoDriving;
  private static final Logger logger = LoggerFactory.getLogger(setAutoDrivingCommand.class);

  public setAutoDrivingCommand(DriveSubsystem driveSubsystem, boolean isAutoDriving) {
    this.driveSubsystem = driveSubsystem;
    this.isAutoDriving = isAutoDriving;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.setAutoDriving(isAutoDriving);
    logger.info("SetAutoDriving: {}", isAutoDriving);
  }
}
