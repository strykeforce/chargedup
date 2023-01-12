package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public ToggleIntakeCommand(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        if (intakeSubsystem.getIsIntakeExtended()) intakeSubsystem.retractClosedLoop();
        else intakeSubsystem.extendClosedLoop();
    }
}
