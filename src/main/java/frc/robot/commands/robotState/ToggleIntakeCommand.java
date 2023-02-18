package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ToggleIntakeCommand extends InstantCommand {
    private RobotStateSubsystem robotStateSubsystem;

    public ToggleIntakeCommand(RobotStateSubsystem robotStateSubsystem) {
        this.robotStateSubsystem = robotStateSubsystem;
    }

    @Override
    public void initialize() {
        if (robotStateSubsystem.getRobotState() == RobotState.INTAKE_STAGE) {
            robotStateSubsystem.toStow();
        }
        else {
            robotStateSubsystem.toIntake();
        }
    }
}
