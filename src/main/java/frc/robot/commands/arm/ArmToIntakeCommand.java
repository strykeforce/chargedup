package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class ArmToIntakeCommand extends CommandBase {
  private ArmSubsystem armSubsystem;

  public ArmToIntakeCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.toIntakePos();
  }

  @Override
  public boolean isFinished() {
      return armSubsystem.getCurrState() == ArmState.INTAKE;
  }
}
