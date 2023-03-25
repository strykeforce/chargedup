package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class InterruptDriveCommand extends InstantCommand {
  private Logger logger = LoggerFactory.getLogger(InterruptDriveCommand.class);

  public InterruptDriveCommand(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    logger.info("AutoDrive Deadman switch released.");
  }
}
