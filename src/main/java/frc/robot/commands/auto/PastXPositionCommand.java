package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PastXPositionCommand extends CommandBase {
    
    private final DriveSubsystem driveSubsystem;
    private double xPose;

    public PastXPositionCommand(DriveSubsystem driveSubsystem, double xPose) {
        addRequirements(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        this.xPose = xPose;
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.getPoseMeters().getX() <= xPose;
    }
}
