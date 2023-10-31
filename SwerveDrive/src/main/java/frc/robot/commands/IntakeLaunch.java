// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.subsystems.IntakeLaunchSubsystem;

public class IntakeLaunch extends CommandBase {

  // Intake mode for the Intake Launch
  private IntakeMode mode;

  /**
   * Creates an IntakeLaunch command
   * 
   * @param mode - mode for the command to use
   */
  public IntakeLaunch(IntakeMode mode) {
    this.mode = mode;
    // adds the subsytem requirement to ensure that only one IntakeLaunch command
    // executes at a time
    addRequirements(IntakeLaunchSubsystem.getInstance());
  }

  /**
   * Initalizes the command (not nessasary)
   */
  @Override
  public void initialize() {
  }

  /**
   * Executes the command to set the speed of the front and back intake motors
   */
  @Override
  public void execute() {
    double frontSpeed = IntakeConstants.INTAKE_FRONT_SPEEDS[this.mode.getIndex()];
    double backSpeed = IntakeConstants.INTAKE_BACK_SPEEDS[this.mode.getIndex()];
    IntakeLaunchSubsystem.getInstance().setSpeeds(frontSpeed, backSpeed);
  }

  /**
   * Runs when the command is ended
   * Sets the speed of the intake motors to zero
   * 
   * @param interrupted
   */
  @Override
  public void end(boolean interrupted) {
    IntakeLaunchSubsystem.getInstance().setSpeeds(0, 0);
  }

  /**
   * Returns when the command should end
   * 
   * @return false -- should never end unless interrupted (by button released)
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
