package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class AutonFloorIntakeCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;

  public AutonFloorIntakeCommand(
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(armSubsystem);
    addRequirements(intakeSubsystem);
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
