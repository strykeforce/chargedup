package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SetVisionUpdateCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private boolean isUpdating;
  private static final Logger logger = LoggerFactory.getLogger(ToggleUpdateWithVisionCommand.class);

  public SetVisionUpdateCommand(DriveSubsystem driveSubsystem, boolean isUpdating) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.isUpdating = isUpdating;
  }

  @Override
  public void initialize() {
    driveSubsystem.visionUpdates = isUpdating;
    logger.info("Do Vision Updates True: {}", driveSubsystem.visionUpdates);
  }
}
