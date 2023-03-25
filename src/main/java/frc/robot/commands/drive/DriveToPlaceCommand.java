package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import java.util.ArrayList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveToPlaceCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveToPlaceCommand.class);
  private Rotation2d desiredHeading;
  private Trajectory place;
  private boolean isShelf;
  private TargetCol targetCol;
  private boolean isBlue;

  public DriveToPlaceCommand(
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
    timer.start();
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.isBlueAlliance()) desiredHeading = new Rotation2d(0.0);
    else desiredHeading = new Rotation2d(Math.PI);
    driveSubsystem.setEnableHolo(true);
    driveSubsystem.resetHolonomicController();
    // driveSubsystem.grapherTrajectoryActive(true);
    // driveSubsystem.lockZero();

    logger.info("Moving to place");
    TrajectoryConfig config = new TrajectoryConfig(2.5, 1.5);
    config.setEndVelocity(0);
    config.setStartVelocity(0.0);
    ArrayList<Translation2d> points = new ArrayList<>();
    Pose2d endPose = new Pose2d();
    if (!isShelf)
      endPose =
          robotStateSubsystem.getAutoPlaceDriveTarget(
              driveSubsystem.getPoseMeters().getY(), targetCol);
    else endPose = robotStateSubsystem.getShelfPosAutoDrive(targetCol, isBlue);

    points.add(
        new Translation2d(
            (driveSubsystem.getPoseMeters().getX() + endPose.getX()) / 2,
            (driveSubsystem.getPoseMeters().getY() + endPose.getY()) / 2));
    Pose2d start =
        new Pose2d(
            new Translation2d(
                driveSubsystem.getPoseMeters().getX(), driveSubsystem.getPoseMeters().getY()),
            new Rotation2d(robotStateSubsystem.isBlueAlliance() ? Math.PI : 0.0));
    driveSubsystem.visionUpdates = false;
    place = TrajectoryGenerator.generateTrajectory(start, points, endPose, config);
    timer.reset();
    driveSubsystem.grapherTrajectoryActive(true);
  }

  @Override
  public void execute() {
    driveSubsystem.calculateController(place.sample(timer.get()), desiredHeading);
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
    driveSubsystem.autoDriving = false;
    // driveSubsystem.visionUpdates = true;
    // driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}", timer.get());
  }
}
