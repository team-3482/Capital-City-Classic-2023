// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakePivot extends CommandBase {

  // stores the target position of Pivot
  private int targetPosition;

  /**
   * Creates a new instance of Pivot Intake Command
   */
  public IntakePivot(IntakeMode location) {

    // sets the target position of the command to the respective value
    this.targetPosition = IntakeConstants.INTAKE_PIVOT_POSITIONS[location.getIndex()];
    // requires the Intake subsytem in order to ensure only one IntakePivot command
    // at a time
    this.addRequirements(IntakePivotSubsystem.getInstance());
  }

  /**
   * Initalizes the command (not nessasary)
   */
  @Override
  public void initialize() {
  }

  /**
   * Executes the Pivot to go to target position
   */
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Pivot Moving", isFinished());
    IntakePivotSubsystem.getInstance().setPivot(targetPosition);
  }

  /**
   * Runs when the command is ended
   * Sets the position of the pivot back to the origin position
   * 
   * @param interrupted
   */
  @Override
  public void end(boolean interrupted) {
    IntakePivotSubsystem.getInstance().setPivot(IntakeConstants.INTAKE_PIVOT_POSITIONS[IntakeMode.NONE.getIndex()]);
    SmartDashboard.putBoolean("Pivot Moving", false);
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
