package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PathData;

public class DriveAutonCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Trajectory trajectory;
  private final Timer timer = new Timer();
  private final Rotation2d robotHeading;
  private boolean lastPath;
  private String trajectoryName;
  private boolean resetOdometry;

  public DriveAutonCommand(
      DriveSubsystem driveSubsystem,
      String trajectoryName,
      boolean lastPath,
      boolean resetOdometry) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.lastPath = lastPath;
    this.resetOdometry = resetOdometry;
    PathData pathdata = driveSubsystem.generateTrajectory(trajectoryName);
    trajectory = pathdata.trajectory;
    robotHeading = pathdata.targetYaw;
    this.trajectoryName = trajectoryName;
    timer.start();
  }

  @Override
  public void initialize() {
    driveSubsystem.setEnableHolo(true);
    Pose2d initialPose = trajectory.getInitialPose();
    if (resetOdometry)
      driveSubsystem.resetOdometry(
          new Pose2d(initialPose.getTranslation(), driveSubsystem.getGyroRotation2d()));
    driveSubsystem.resetHolonomicController();
    // driveSubsystem.grapherTrajectoryActive(true);
    timer.reset();
    // logger.info("Begin Trajectory: {}", trajectoryName);

  }

  @Override
  public void execute() {
    Trajectory.State desiredState = trajectory.sample(timer.get());
    driveSubsystem.calculateController(desiredState, robotHeading);
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

    // driveSubsystem.grapherTrajectoryActive(false);
    // logger.info("End Trajectory {}: {}", trajectoryName, timer.get());
  }
}
