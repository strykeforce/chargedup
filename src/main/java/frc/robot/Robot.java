// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SetAllianceCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private static final Logger logger = LoggerFactory.getLogger(Robot.class);

  private RobotContainer m_robotContainer;
  private boolean haveAlliance;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    haveAlliance = false;

    Shuffleboard.getTab("Match")
        .add("SetAllianceRed", new SetAllianceCommand(Alliance.Red, m_robotContainer))
        .withPosition(2, 0)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .add("SetAllianceBlue", new SetAllianceCommand(Alliance.Blue, m_robotContainer))
        .withPosition(2, 1)
        .withSize(1, 1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // CommandScheduler.getInstance()
    //     .onCommandInitialize(command -> logger.info("{} initialized", command.getName()));
    if (!haveAlliance) {
      Alliance alliance = DriverStation.getAlliance();
      if (alliance != Alliance.Invalid) {
        haveAlliance = true;
        m_robotContainer.setAllianceColor(alliance);
        logger.info("Set Alliance {}", alliance);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
