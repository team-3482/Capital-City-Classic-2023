// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.subsystems.IntakePivotSubsystem;

/** An example command that uses an example subsystem. */
public class IntakePivot extends CommandBase {

  private int targetPosition;

  public IntakePivot(IntakeMode location) {

    this.targetPosition = IntakeConstants.INTAKE_PIVOT_POSITIONS[location.getIndex()];
    this.addRequirements(IntakePivotSubsystem.getInstance());
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Pivot Moving", isFinished());
    IntakePivotSubsystem.getInstance().setPivot(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
    IntakePivotSubsystem.getInstance().setPivot(IntakeConstants.INTAKE_PIVOT_POSITIONS[IntakeMode.NONE.getIndex()]);
    SmartDashboard.putBoolean("Pivot Moving", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
