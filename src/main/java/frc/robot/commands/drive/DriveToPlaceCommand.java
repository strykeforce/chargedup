package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveToPlaceCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private Rotation2d robotHeading;

  public DriveToPlaceCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue)
      robotHeading = new Rotation2d(-1.0, 0.0);
    else robotHeading = new Rotation2d(0.0, 0.0);
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    driveSubsystem.resetHolonomicController();
    // driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    logger.info("Moving to place");
  }

  @Override
  public void execute() {
    // FIX ME
    // I HATE PHOTONVISION
    // PUT IN CONSTANTS
    State autoDrive =
        new State(
            3.0,
            1.0,
            0.1,
            robotStateSubsystem.getAutoPlaceDriveTarget(driveSubsystem.getPoseMeters().getY()),
            1);
    logger.info(
        "Going to ( " + autoDrive.poseMeters.getX() + " , " + autoDrive.poseMeters.getY() + " )");
    driveSubsystem.calculateController(autoDrive, robotHeading);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);
    driveSubsystem.drive(0, 0, 0);
    // driveSubsystem.grapherTrajectoryActive(false);
    logger.info("Done auto drive");
  }
}
