// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakePivotSubsystem extends SubsystemBase {

  private static IntakePivotSubsystem instance;

  public static IntakePivotSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakePivotSubsystem();
    }
    return instance;
  }

  private WPI_TalonFX intakePivot;

  public IntakePivotSubsystem() {
    this.intakePivot = new WPI_TalonFX(IntakeConstants.INTAKE_PIVOR_ID, IntakeConstants.ALTERNATE_CAN_BUS);
    // Configure Motion Magic

    this.intakePivot.configFactoryDefault();
    this.intakePivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, IntakeConstants.PID_LOOP_IDX,
        IntakeConstants.TIMEOUT_MS);

    this.intakePivot.configNeutralDeadband(0.001, IntakeConstants.TIMEOUT_MS);
    this.intakePivot.setSensorPhase(false);
    this.intakePivot.setInverted(false);
    this.intakePivot.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, IntakeConstants.TIMEOUT_MS);
    this.intakePivot.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, IntakeConstants.TIMEOUT_MS);

    /* Set the peak and nominal outputs */
    this.intakePivot.configNominalOutputForward(0, IntakeConstants.TIMEOUT_MS);
    this.intakePivot.configNominalOutputReverse(0, IntakeConstants.TIMEOUT_MS);
    this.intakePivot.configPeakOutputForward(1, IntakeConstants.TIMEOUT_MS);
    this.intakePivot.configPeakOutputReverse(-1, IntakeConstants.TIMEOUT_MS);

    /* Set Motion Magic gains in slot0 - see documentation */
    this.intakePivot.selectProfileSlot(IntakeConstants.PID_SLOT_IDX, IntakeConstants.PID_LOOP_IDX);
    this.intakePivot.config_kF(IntakeConstants.PID_SLOT_IDX, IntakeConstants.GAINS.kF, IntakeConstants.TIMEOUT_MS);
    this.intakePivot.config_kP(IntakeConstants.PID_SLOT_IDX, IntakeConstants.GAINS.kP, IntakeConstants.TIMEOUT_MS);
    this.intakePivot.config_kI(IntakeConstants.PID_SLOT_IDX, IntakeConstants.GAINS.kI, IntakeConstants.TIMEOUT_MS);
    this.intakePivot.config_kD(IntakeConstants.PID_SLOT_IDX, IntakeConstants.GAINS.kD, IntakeConstants.TIMEOUT_MS);

    /* Set acceleration and vcruise velocity - see documentation */
    this.intakePivot.configMotionCruiseVelocity(IntakeConstants.CRUISE_SPEED, IntakeConstants.TIMEOUT_MS); // 1500
    this.intakePivot.configMotionAcceleration(IntakeConstants.CRUISE_ACCELERATION, IntakeConstants.TIMEOUT_MS); // 600

    /* Zero the sensor once on robot boot up */
    this.intakePivot.setSelectedSensorPosition(0, IntakeConstants.PID_LOOP_IDX, IntakeConstants.TIMEOUT_MS);

    this.intakePivot.configMotionSCurveStrength(IntakeConstants.MOTION_MAGIC_SMOOTHING);
  }

  public void setPivot(double targetPosition) {

    this.intakePivot.set(TalonFXControlMode.MotionMagic, targetPosition);

    SmartDashboard.putNumber("Intake Current Pivot", this.intakePivot.get());
    SmartDashboard.putNumber("Intake Target Pivot", targetPosition);
  }

  public double getPivot() {
    return this.intakePivot.get();
  }
}
