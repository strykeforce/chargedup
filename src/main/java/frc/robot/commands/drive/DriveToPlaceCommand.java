package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import java.util.ArrayList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveToPlaceCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private Rotation2d robotHeading;
  private Trajectory place;

  public DriveToPlaceCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;

    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue) robotHeading = new Rotation2d(0.0);
    else robotHeading = new Rotation2d(Math.toRadians(180));
    timer.start();
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    driveSubsystem.resetHolonomicController();
    // driveSubsystem.grapherTrajectoryActive(true);
    // driveSubsystem.lockZero();

    logger.info("Moving to place");
    TrajectoryConfig config = new TrajectoryConfig(2.5, 1.5);
    config.setEndVelocity(0);
    config.setStartVelocity(0.0);
    ArrayList<Translation2d> points = new ArrayList<>();
    Pose2d endPose =
        robotStateSubsystem.getAutoPlaceDriveTarget(driveSubsystem.getPoseMeters().getY());
    points.add(
        new Translation2d(
            (driveSubsystem.getPoseMeters().getX() + endPose.getX()) / 2,
            (driveSubsystem.getPoseMeters().getY() + endPose.getY()) / 2));
    Pose2d start =
        new Pose2d(
            new Translation2d(
                driveSubsystem.getPoseMeters().getX(), driveSubsystem.getPoseMeters().getY()),
            new Rotation2d(
                robotStateSubsystem.getAllianceColor() == Alliance.Blue ? Math.toRadians(180) : 0));
    place = TrajectoryGenerator.generateTrajectory(start, points, endPose, config);
    driveSubsystem.visionUpdates = false;
    timer.reset();
    driveSubsystem.grapherTrajectoryActive(true);
  }

  @Override
  public void execute() {
    driveSubsystem.calculateController(place.sample(timer.get()), robotHeading);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(place.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.grapherTrajectoryActive(false);
    driveSubsystem.setEnableHolo(false);
    driveSubsystem.drive(0, 0, 0);
    // driveSubsystem.visionUpdates = true;
    // driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}", timer.get());
  }
}
