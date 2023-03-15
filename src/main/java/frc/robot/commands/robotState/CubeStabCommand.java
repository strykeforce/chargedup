package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotStateSubsystem;

public class CubeStabCommand extends CommandBase {
    private RobotStateSubsystem robotStateSubsystem;

    public CubeStabCommand(RobotStateSubsystem robotStateSubsystem) {
        addRequirements(robotStateSubsystem);

        this.robotStateSubsystem = robotStateSubsystem;
    }
}
