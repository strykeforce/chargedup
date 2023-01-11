// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

// Requires swerve migration to new Pose2D
//import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final static double kJoystickDeadband = 0.1;

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem;
  private final Joystick joystick = new Joystick(0);

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();

    driveSubsystem.setDefaultCommand(new RunCommand(
      () -> {
        double vx = getLeftX() * -DriveConstants.kMaxSpeedMetersPerSecond;
        double vy = getLeftY() * -DriveConstants.kMaxSpeedMetersPerSecond;
        double omega = getRightY() * -DriveConstants.kMaxOmega;
        driveSubsystem.drive(vx, vy, omega);
      }, driveSubsystem));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick, InterlinkButton.RESET.id).whenPressed(driveSubsystem::resetGyro, driveSubsystem);
    // Requires swerve migration to new Pose2D
    //new JoystickButton(joystick, InterlinkButton.HAMBURGER.id).whenPressed(() -> {driveSubsystem.resetOdometry(new Pose2d());},driveSubsystem);
  }

  //public Command getAutonomousCommand() {}

  //Interlink Controller Mapping
  public enum Axis{
    RIGHT_X(1),
    RIGHT_Y(0),
    LEFT_X(2),
    LEFT_Y(5),
    TUNER(6),
    LEFT_BACK(4),
    RIGHT_BACK(3);

    private final int id;
    Axis(int id){
      this.id = id;
    }
  }

  public enum Shoulder{
    RIGHT_DOWN(2),
    LEFT_DOWN(4),
    LEFT_UP(5);

    private final int id;
    Shoulder(int id){
      this.id = id;
    }
  }

  public enum Toggle{
    LEFT_TOGGLE(1);
    
    private final int id;
    Toggle(int id){
      this.id = id;
    }
  }

  public enum InterlinkButton{
    RESET(3),
    HAMBURGER(14),
    X(15),
    UP(16),
    DOWN(17);

    private final int id;
    InterlinkButton(int id){
      this.id = id;
    }
  }

  public enum Trim{
    LEFT_Y_POS(7),
    LEFT_Y_NEG(6),
    LEFT_X_POS(8),
    LEFT_X_NEG(9),
    RIGHT_X_POS(10),
    RIGHT_X_NEG(11),
    RIGHT_Y_POS(12),
    RIGHT_Y_NEG(13);

    private final int id;
    Trim(int id){
      this.id = id;
    }
  }

  public double getLeftX(){
    double val = joystick.getRawAxis(Axis.LEFT_X.id);
    if(Math.abs(val) < kJoystickDeadband) return 0.0;
    return val;
  }

  public double getLeftY(){
    double val = joystick.getRawAxis(Axis.LEFT_Y.id);
    if(Math.abs(val) < kJoystickDeadband) return 0.0;
    return val;
  }

  public double getRightY(){
    double val = joystick.getRawAxis(Axis.RIGHT_Y.id);
    if(Math.abs(val) < kJoystickDeadband) return 0.0;
    return val;
  }
}