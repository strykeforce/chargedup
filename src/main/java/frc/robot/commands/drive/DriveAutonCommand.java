package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PathData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DriveAutonCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private Trajectory trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(DriveAutonCommand.class);
  private Rotation2d robotHeading;
  private boolean lastPath;
  private String trajectoryName;
  private boolean resetOdometry;
  private boolean trajectoryGenerated = false;

  public DriveAutonCommand(
      DriveSubsystem driveSubsystem,
      String trajectoryName,
      boolean lastPath,
      boolean resetOdometry) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.lastPath = lastPath;
    this.resetOdometry = resetOdometry;
    this.trajectoryName = trajectoryName;
    generateTrajectory();
    timer.start();
  }

  public void generateTrajectory() {
    PathData pathdata = driveSubsystem.generateTrajectory(trajectoryName);
    trajectory = pathdata.trajectory;
    robotHeading = pathdata.targetYaw;
    logger.info("trajectory generated");
    trajectoryGenerated = true;
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    Pose2d initialPose = trajectory.getInitialPose();
    if (resetOdometry)
      driveSubsystem.resetOdometry(
          new Pose2d(initialPose.getTranslation(), driveSubsystem.getGyroRotation2d()));
    driveSubsystem.resetHolonomicController();
    driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    logger.info("Begin Trajectory: {}", trajectoryName);
  }

  @Override
  public void execute() {
    if (trajectoryGenerated) {
      Trajectory.State desiredState = trajectory.sample(timer.get());
      driveSubsystem.calculateController(desiredState, robotHeading);
    } else logger.error("trajectory not generated");
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setEnableHolo(false);

    if (!lastPath) {
      driveSubsystem.calculateController(
          trajectory.sample(trajectory.getTotalTimeSeconds()), robotHeading);
    } else {
      driveSubsystem.drive(0, 0, 0);
    }

    driveSubsystem.grapherTrajectoryActive(false);
    logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
    trajectoryGenerated = false;
  }
}
