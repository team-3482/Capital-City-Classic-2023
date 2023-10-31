// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeLaunchSubsystem extends SubsystemBase {

  private static IntakeLaunchSubsystem instance;

  public static IntakeLaunchSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeLaunchSubsystem();
    }
    return instance;
  }

  private CANSparkMax intakeFront;
  private CANSparkMax intakeBack;

  public IntakeLaunchSubsystem() {
    this.intakeFront = new CANSparkMax(IntakeConstants.INTAKE_FRONT_SPARK_MAX_ID, MotorType.kBrushless);
    this.intakeBack = new CANSparkMax(IntakeConstants.INTAKE_BACK_SPARK_MAX_ID, MotorType.kBrushless);
  }

  public void setSpeeds(double frontSpeed, double backSpeed) {

    this.intakeFront.set(frontSpeed);
    this.intakeBack.set(backSpeed);

    SmartDashboard.putNumber("Intake Front Speed", frontSpeed);
    SmartDashboard.putNumber("Intake Back Speed", backSpeed);
  }
}
