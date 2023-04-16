package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveStates;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
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
  private boolean doYawAdjust;
  private double yawAdjustBy;

  public AutoPlaceCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      boolean doYawAdjust,
      double yawAdjustBy) {
    addRequirements(driveSubsystem, armSubsystem, handSubsystem);
    this.doYawAdjust = doYawAdjust;
    this.driveSubsystem = driveSubsystem;
    this.yawAdjustBy = yawAdjustBy;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.setDriveState(DriveStates.IDLE);
    // driveSubsystem.setAutoDriving(true);
    // First autodrive.
    logger.info(
        "Starting Autoplace, Level: {}, Position(Col): {}, isShelf: {}",
        robotStateSubsystem.getTargetLevel().name(),
        robotStateSubsystem.getTargetCol().name(),
        (robotStateSubsystem.getGamePiece() == GamePiece.NONE));
    driveSubsystem.setDoYawAdjust(doYawAdjust, yawAdjustBy);
    if (robotStateSubsystem.getGamePiece() == GamePiece.NONE) robotStateSubsystem.toAutoShelf();
    else robotStateSubsystem.toAutoScore();
    // robotStateSubsystem.toAutoDrive();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return (driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FINISHED
        || driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FAILED);
  }

  @Override
  public void end(boolean interrupted) {
    robotStateSubsystem.endAutoPlace(interrupted);
    driveSubsystem.setAutoDriving(false);
    if (driveSubsystem.currDriveState != DriveStates.AUTO_DRIVE_FINISHED
        && driveSubsystem.currDriveState != DriveStates.AUTO_DRIVE_FAILED) {
      logger.info("Drive: {} -> AUTO_DRIVE_FINISHED", driveSubsystem.currDriveState);
      driveSubsystem.currDriveState = DriveStates.AUTO_DRIVE_FINISHED;
    }
  }
}
