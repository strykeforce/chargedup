package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoPlaceCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(AutoPlaceCommand.class);
  private boolean isShelf;
  private TargetCol targetCol;
  private boolean isBlue;
  private TargetLevel targetLevel;

  public AutoPlaceCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem, 
      boolean isShelf,
      TargetCol targetCol,
      boolean isBlue, TargetLevel targetLevel) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.isShelf = isShelf;
    this.targetCol = targetCol;
    this.isBlue = isBlue;
    this.targetLevel = targetLevel;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //First autodrive.
    logger.info("Starting Autoplace, Level: {}", targetLevel.name());
    robotStateSubsystem.setTargetLevel(targetLevel);
    robotStateSubsystem.toAutoDrive(isShelf, targetCol, isBlue);
  }

  @Override
  public boolean isFinished() {
    return driveSubsystem.isAutoDriveFinished() && robotStateSubsystem.isAutoPlaceFinished();
  }

  @Override
  public void end(boolean interrupted) {
    robotStateSubsystem.endAutoPlace();
  }
}
