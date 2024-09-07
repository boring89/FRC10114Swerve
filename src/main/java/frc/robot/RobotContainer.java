// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwervejoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort); 
  //private final PS4Controller driverJoystick = new PS4Controller(OIConstants.kDriverControllerPort);
  //XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwervejoystickCmd(         
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    new InstantCommand();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 1).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(),swerveSubsystem));
  }

  Command getAutonomousCommand() {
    return null;
  }
  
}
