package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveStates;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoBalanceCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private boolean isOnAllianceSide;
  private static final Logger logger = LoggerFactory.getLogger(AutoBalanceCommand.class);

  public AutoBalanceCommand(
      boolean isOnAllianceSide,
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    this.isOnAllianceSide = isOnAllianceSide;
    addRequirements(driveSubsystem, robotStateSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.setDriveState(DriveStates.IDLE);
    logger.info("Starting Autobalance isOnAllianceSideCStation: {}", isOnAllianceSide);
    robotStateSubsystem.toAutoBalance(isOnAllianceSide);
  }

  @Override
  public void execute() {
    // if (driveSubsystem.currDriveState == DriveStates.AUTO_BALANCE_FINISHED) {
    //   driveSubsystem.autoBalanceReadjust = false;
    //   driveSubsystem.autoBalanceGyroActive = false;
    // }
  }

  @Override
  public boolean isFinished() {
    return false; // driveSubsystem.currDriveState == DriveStates.AUTO_BALANCE_FINISHED;
  }

  @Override
  public void end(boolean interrupted) {
    logger.info("Autobalance Finished Interrupted: {}", interrupted);
    driveSubsystem.autoBalanceReadjust = false;
    driveSubsystem.autoBalanceGyroActive = false;
  }
}
