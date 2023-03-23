package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveStates;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class PulseAutoBalancePulseCommand extends CommandBase {
  private static final Logger logger = LoggerFactory.getLogger(PulseAutoBalancePulseCommand.class);
  private RobotStateSubsystem robotStateSubsystem;
  private Boolean isOnAllianceSide;
  private DriveSubsystem driveSubsystem;

  public PulseAutoBalancePulseCommand(
      Boolean isOnAllianceSide,
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.isOnAllianceSide = isOnAllianceSide;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.setDriveState(DriveStates.IDLE);
    logger.info("Starting Autobalance isOnAllianceSideCStation: {}", isOnAllianceSide);
    robotStateSubsystem.toPulseAutoBalance(isOnAllianceSide);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    logger.info("Autobalance Finished Interrupted: {}", interrupted);
  }
}
