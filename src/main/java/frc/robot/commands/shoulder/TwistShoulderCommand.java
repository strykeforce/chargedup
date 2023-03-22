package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ShoulderSubsystem;

public class TwistShoulderCommand extends CommandBase {
  private ArmSubsystem armSubsystem;
  private ShoulderSubsystem shoulderSubsystem;
  private int sign;

  public TwistShoulderCommand(
      ShoulderSubsystem shoulderSubsystem, ArmSubsystem armSubsystem, int sign) {
    addRequirements(shoulderSubsystem, armSubsystem);
    this.shoulderSubsystem = shoulderSubsystem;
    this.armSubsystem = armSubsystem;
    this.sign = sign;
  }

  @Override
  public void execute() {
    if (armSubsystem.getCurrState() == ArmState.TWIST_SHOULDER
        && armSubsystem.canTwist(ShoulderConstants.kTwistBy * sign))
      armSubsystem.twistShoulder(sign * ShoulderConstants.kTwistBy);
  }
}
