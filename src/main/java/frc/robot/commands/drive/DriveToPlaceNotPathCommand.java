package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveToPlaceNotPathCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private Rotation2d robotHeading;
  private Trajectory place;

  public DriveToPlaceNotPathCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;

    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue) robotHeading = new Rotation2d(0.0);
    else robotHeading = new Rotation2d(Math.toRadians(180));
    // timer.start();
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    driveSubsystem.resetHolonomicController();
    driveSubsystem.driveToPose();
    // driveSubsystem.grapherTrajectoryActive(true);
    // driveSubsystem.lockZero();
    // timer.reset();
    logger.info("Moving to place");
    ;
  }

  @Override
  public boolean isFinished() {
    return !driveSubsystem.autoDriving;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDriving = false;
    driveSubsystem.setEnableHolo(false);
    driveSubsystem.drive(0, 0, 0);
    // driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}"); // timer.get());
  }
}
