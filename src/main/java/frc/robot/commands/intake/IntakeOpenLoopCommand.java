package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOpenLoopCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private double percentOutput;

  public IntakeOpenLoopCommand(IntakeSubsystem intakeSubsystem, double percentOutput) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.percentOutput = percentOutput;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setIntakeSpeed(percentOutput);
    intakeSubsystem.setRollerSpeed(percentOutput);
  }
}
