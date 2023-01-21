// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsytems.ElbowSubsystem;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private final ElbowSubsystem elbowSubsystem;
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  public RobotContainer() {
    elbowSubsystem = new ElbowSubsystem();

    configureTelemetry();
    configureBindings();
  }

  private void configureBindings() {}

  private void configureTelemetry() {
    elbowSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
