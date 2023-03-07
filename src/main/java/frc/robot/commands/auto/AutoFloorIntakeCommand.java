package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class AutoFloorIntakeCommand extends CommandBase {

  private final RobotStateSubsystem robotStateSubsystem;
  private final ArmSubsystem armSubsystem;

  public AutoFloorIntakeCommand(
      RobotStateSubsystem robotStateSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem) {
    addRequirements(intakeSubsystem, armSubsystem, handSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toIntake();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.PICKUP_FROM_INTAKE;
  }
}
