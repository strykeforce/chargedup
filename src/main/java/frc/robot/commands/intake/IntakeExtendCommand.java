package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeExtendCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private boolean doExtend;

  public IntakeExtendCommand(IntakeSubsystem intakeSubsystem, boolean doExtend) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.doExtend = doExtend;
  }

  @Override
  public void initialize() {
    if (doExtend) intakeSubsystem.extendClosedLoop();
    else intakeSubsystem.retractClosedLoop();
  }
}
