package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoPlaceCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(AutoPlaceCommand.class);
  private boolean isShelf;
  private TargetCol targetCol;
  private boolean isBlue;

  public AutoPlaceCommand(DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    // First autodrive.
    logger.info("Starting Autoplace, Level: {}", robotStateSubsystem.getTargetLevel().name());
    // robotStateSubsystem.setTargetLevel(robotStateSubsystem.getTargetLevel());
    robotStateSubsystem.toAutoDrive();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return driveSubsystem.isAutoDriveFinished() && robotStateSubsystem.isAutoPlaceFinished();
  }

  @Override
  public void end(boolean interrupted) {
    robotStateSubsystem.endAutoPlace(interrupted);
  }
}
