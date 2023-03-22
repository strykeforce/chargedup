package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ShoulderSubsystem;

public class StopTwistCommand extends CommandBase {
  private ShoulderSubsystem shoulderSubsystem;
  private ArmSubsystem armSubsystem;

  public StopTwistCommand(ShoulderSubsystem shoulderSubsystem, ArmSubsystem armSubsystem) {
    addRequirements(shoulderSubsystem, armSubsystem);
    this.armSubsystem = armSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
  }

  @Override
  public void initialize() {
    if (armSubsystem.getCurrState() == ArmState.TWIST_SHOULDER) armSubsystem.twistShoulder(0.0);
  }
}
