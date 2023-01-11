package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public ToggleIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        if (intakeSubsystem.getIsIntakeExtended()) {
            intakeSubsystem.retractClosedLoop();
            intakeSubsystem.setIntakeSpeed(0);
            intakeSubsystem.setRollerSpeed(0);
        } else {
            intakeSubsystem.extendClosedLoop();
            intakeSubsystem.setIntakeSpeed(1);
            intakeSubsystem.setRollerSpeed(1);
        }
    }
}
