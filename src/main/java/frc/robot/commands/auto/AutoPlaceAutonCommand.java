package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveStates;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoPlaceAutonCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(AutoPlaceAutonCommand.class);
  private boolean isShelf;
  private TargetCol targetCol;
  private boolean isBlue;
  private boolean doYawAdjust;
  private double yawAdjustBy;

  public AutoPlaceAutonCommand(
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
    logger.info(
        "Starting Autoplace in Auton: Level: {}, Position(Col): {}",
        robotStateSubsystem.getTargetLevel().name(),
        robotStateSubsystem.getTargetCol().name());
    driveSubsystem.setDoYawAdjust(doYawAdjust, yawAdjustBy);
    robotStateSubsystem.toAutoScore();
  }

  @Override
  public boolean isFinished() {
    return (driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FINISHED
        || driveSubsystem.currDriveState == DriveStates.AUTO_DRIVE_FAILED);
  }

  @Override
  public void end(boolean interrupted) {
    robotStateSubsystem.endAutoPlace(interrupted);
    driveSubsystem.setAutoDriving(false);
  }
}
