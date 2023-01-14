package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeExtendedCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  public ToggleIntakeExtendedCommand(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    if (intakeSubsystem.getIsIntakeExtended()) intakeSubsystem.retractIntake();
    else intakeSubsystem.startIntaking();
  }
}
