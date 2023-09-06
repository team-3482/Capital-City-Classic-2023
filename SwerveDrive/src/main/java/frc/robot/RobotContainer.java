// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // Instance of the Swerve Subsystem
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // Instance of the Controller used to drive the robot
  private CommandXboxController driveController = new CommandXboxController(
      Constants.ControllerConstants.CONTROLLER_ID);

  /**
   * Creates an instance of the robot controller
   */
  public RobotContainer() {

    // sets the default command to being driving swerve
    swerveSubsystem.setDefaultCommand(new SwerveDrive(swerveSubsystem,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> driveController.getRawAxis(Constants.ControllerConstants.DRIVE_ROT_AXIS),
        () -> driveController.getHID().getAButton()));

    configureBindings();
  }

  /**
   * Configures the button bindings of the controller
   */
  private void configureBindings() {
    driveController.y().whileTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));
  }

}
