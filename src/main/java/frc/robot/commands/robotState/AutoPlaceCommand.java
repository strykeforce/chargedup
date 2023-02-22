package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drive.DriveAutonCommand;
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

  public AutoPlaceCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      boolean isShelf,
      TargetCol targetCol,
      boolean isBlue) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.isShelf = isShelf;
    this.targetCol = targetCol;
    this.isBlue = isBlue;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //First autodrive.
    robotStateSubsystem.toAutoDrive(isShelf, targetCol, isBlue);
  }

  @Override
  public boolean isFinished() {
    return driveSubsystem.isAutoDriveFinished(); //+ OTher;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
