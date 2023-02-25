package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ToggleUpdateWithVisionCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(ToggleUpdateWithVisionCommand.class);

  public ToggleUpdateWithVisionCommand(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.visionUpdates = !driveSubsystem.visionUpdates;
    logger.info("Do Vision Updates True: {}", driveSubsystem.visionUpdates);
  }
}
