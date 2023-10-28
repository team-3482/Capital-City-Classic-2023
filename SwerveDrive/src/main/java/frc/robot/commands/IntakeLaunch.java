// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.subsystems.IntakeLaunchSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeLaunch extends CommandBase {

  private IntakeMode mode;

  public IntakeLaunch(IntakeMode mode) {

    this.mode = mode;
    addRequirements(IntakeLaunchSubsystem.getInstance());
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double frontSpeed = IntakeConstants.INTAKE_FRONT_SPEEDS[this.mode.getIndex()];
    double backSpeed = IntakeConstants.INTAKE_BACK_SPEEDS[this.mode.getIndex()];
    IntakeLaunchSubsystem.getInstance().setSpeeds(frontSpeed, backSpeed);

  }

  @Override
  public void end(boolean interrupted) {
    IntakeLaunchSubsystem.getInstance().setSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
