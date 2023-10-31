// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.commands.IntakePivot;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // Singleton design pattern
  public static RobotContainer instance;

  /**
   * Gets the instance of the RobotContainer
   * 
   * @return instance
   */
  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  // Instance of the Swerve Subsystem
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // Instance of the controller used to drive the robot
  private CommandXboxController driveController;
  // Instance of the controller used to control intake
  private CommandXboxController intakeController;

  /**
   * Creates an instance of the robot controller
   */
  public RobotContainer() {

    this.swerveSubsystem = new SwerveSubsystem();
    this.driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER_ID);
    this.intakeController = new CommandXboxController(ControllerConstants.INTAKE_CONTROLLER_ID);
    // sets the default command to being driving swerve
    this.swerveSubsystem.setDefaultCommand(new SwerveDrive(swerveSubsystem,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> driveController.getRawAxis(Constants.ControllerConstants.DRIVE_ROT_AXIS),
        () -> !driveController.getHID().getAButton(),
        () -> driveController.getHID().getRightBumper()));

    configureBindings();
  }

  /**
   * Configures the button bindings of the controllers
   */
  private void configureBindings() {
    driveController.y().whileTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));

    intakeController.y().whileTrue(new IntakePivot(IntakeMode.LOW));
  }
}
