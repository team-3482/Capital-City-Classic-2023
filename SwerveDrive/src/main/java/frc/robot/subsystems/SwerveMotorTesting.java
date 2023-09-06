// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveMotorTesting extends SubsystemBase {

  // private static SwerveMotorTesting instance;

  // public static SwerveMotorTesting getInstance() {
  // if (instance == null) {
  // instance = new SwerveMotorTesting();
  // }
  // return instance;
  // }

  // private CANSparkMax oneDrive;
  // private CANSparkMax oneTurn;
  // private CANCoder oneEncoder;

  // private CANSparkMax twoDrive;
  // private CANSparkMax twoTurn;
  // private CANCoder twoEncoder;

  // private CANSparkMax threeDrive;
  // private CANSparkMax threeTurn;
  // private CANCoder threeEncoder;

  // private CANSparkMax fourDrive;
  // private CANSparkMax fourTurn;
  // private CANCoder fourEncoder;

  // private CommandXboxController controller;

  // private PIDController pidController1;
  // // private PIDSource pidSource1;

  // /** Creates a new SwerveMotorTesting. */
  // public SwerveMotorTesting() {

  // /*
  // * this.oneDrive = new CANSparkMax(Constants.SWERVE_MODULE_ONE_DRIVE,
  // * MotorType.kBrushless);
  // * this.oneTurn = new CANSparkMax(Constants.SWERVE_MODULE_ONE_TURN,
  // * MotorType.kBrushless);
  // * this.oneEncoder = new CANCoder(Constants.SWERVE_MODULE_ONE_ENCODER,
  // * "swerve");
  // *
  // * this.twoDrive = new CANSparkMax(Constants.SWERVE_MODULE_TWO_DRIVE,
  // * MotorType.kBrushless);
  // * this.twoTurn = new CANSparkMax(Constants.SWERVE_MODULE_TWO_TURN,
  // * MotorType.kBrushless);
  // * this.twoEncoder = new CANCoder(Constants.SWERVE_MODULE_TWO_ENCODER,
  // * "swerve");
  // *
  // * this.threeDrive = new CANSparkMax(Constants.SWERVE_MODULE_THREE_DRIVE,
  // * MotorType.kBrushless);
  // * this.threeTurn = new CANSparkMax(Constants.SWERVE_MODULE_THREE_TURN,
  // * MotorType.kBrushless);
  // * this.threeEncoder = new CANCoder(Constants.SWERVE_MODULE_THREE_ENCODER,
  // * "swerve");
  // *
  // * this.fourDrive = new CANSparkMax(Constants.SWERVE_MODULE_FOUR_DRIVE,
  // * MotorType.kBrushless);
  // * this.fourTurn = new CANSparkMax(Constants.SWERVE_MODULE_FOUR_TURN,
  // * MotorType.kBrushless);
  // * this.fourEncoder = new CANCoder(Constants.SWERVE_MODULE_FOUR_ENCODER,
  // * "swerve");
  // *
  // * this.controller = new CommandXboxController(Constants.CONTROLLER_ID);
  // *
  // * this.pidController1 = new PIDController(Constants.KP, Constants.KI,
  // * Constants.KD);
  // *
  // * this.configureBindings(ButtonModes.TestingMotors);
  // */
  // }

  // private void configureBindings(ButtonModes buttonMode) {

  // if (buttonMode == ButtonModes.TestingMotors) {
  // this.controller.a().whileTrue(Commands.startEnd(() -> {
  // this.setDriveMotors(Constants.SPEED);
  // System.out.println("Drive On");
  // }, () -> {
  // this.setDriveMotors(0.0);
  // System.out.println("Drive Off");
  // }, this));

  // this.controller.b().whileTrue(Commands.startEnd(() -> {
  // this.setTurnMotors(Constants.SPEED);
  // System.out.println("Turn On");
  // }, () -> {
  // this.setTurnMotors(0.0);
  // System.out.println("Turn Off");
  // }, this));

  // this.controller.x().whileTrue(Commands.startEnd(() -> {
  // this.setDriveMotors(-1 * Constants.SPEED);
  // System.out.println("Turn On");
  // }, () -> {
  // this.setDriveMotors(0.0);
  // System.out.println("Turn Off");
  // }, this));
  // } else if (buttonMode == ButtonModes.ZeroingTurnEncoders) { // a = one, b =
  // two, y = three, x = four

  // this.controller.a().whileTrue(Commands.startEnd(() -> {
  // resetDriveMotors(oneDrive, oneEncoder, 0);
  // }, () -> {
  // this.oneTurn.set(0);
  // }, this));

  // this.controller.b().whileTrue(Commands.startEnd(() -> {
  // resetDriveMotors(twoTurn, twoEncoder, 0);
  // }, () -> {
  // this.twoTurn.set(0);
  // }, this));

  // this.controller.y().whileTrue(Commands.startEnd(() -> {
  // resetDriveMotors(threeTurn, threeEncoder, 0);
  // }, () -> {
  // this.threeTurn.set(0);
  // }, this));

  // this.controller.x().whileTrue(Commands.startEnd(() -> {
  // resetDriveMotors(fourTurn, fourEncoder, 0);
  // }, () -> {
  // this.fourTurn.set(0);
  // }, this));

  // } else if (buttonMode == ButtonModes.AlignMotors) {

  // }

  // }

  // private void resetDriveMotors(CANSparkMax motor, CANCoder encoder, double
  // setpoint) {
  // if (motor == null || encoder == null) {
  // return;
  // }
  // // motor.set(this.pidController.calculate(encoder.getPosition(), setpoint));
  // }

  // private void setDriveMotors(double speed) {
  // if (Constants.SWERVE_MODULE_ONE_ENABLED) {
  // this.oneDrive.set(speed);
  // }
  // if (Constants.SWERVE_MODULE_TWO_ENABLED) {

  // this.twoDrive.set(speed);
  // }
  // if (Constants.SWERVE_MODULE_THREE_ENABLED) {
  // this.threeDrive.set(speed);
  // }
  // if (Constants.SWERVE_MODULE_FOUR_ENABLED) {
  // this.fourDrive.set(speed);
  // }
  // System.out.println("Speed of Enabled Drive Motors Set To: " + speed);
  // }

  // private void setTurnMotors(double speed) {
  // if (Constants.SWERVE_MODULE_ONE_ENABLED) {
  // this.oneTurn.set(speed);
  // }
  // if (Constants.SWERVE_MODULE_TWO_ENABLED) {

  // this.twoTurn.set(speed);
  // }
  // if (Constants.SWERVE_MODULE_THREE_ENABLED) {
  // this.threeTurn.set(speed);
  // }
  // if (Constants.SWERVE_MODULE_FOUR_ENABLED) {
  // this.fourTurn.set(speed);
  // }
  // System.out.println("Speed of Enabled Turn Motors Set To: " + speed);
  // }

  // public void outputToShuffleboard() {

  // SmartDashboard.putNumber("Module One Encoder Absolute Position",
  // this.oneEncoder.getAbsolutePosition());
  // SmartDashboard.putNumber("Module One Encoder Non-Absolute Position",
  // this.oneEncoder.getPosition());

  // SmartDashboard.putNumber("Module Two Encoder Absolute Position",
  // this.twoEncoder.getAbsolutePosition());
  // SmartDashboard.putNumber("Module Two Encoder Non-Absolute Position",
  // this.twoEncoder.getPosition());

  // SmartDashboard.putNumber("Module Three Encoder Absolute Position",
  // this.threeEncoder.getAbsolutePosition());
  // SmartDashboard.putNumber("Module Three Encoder Non-Absolute Position",
  // this.threeEncoder.getPosition());

  // SmartDashboard.putNumber("Module Four Encoder Absolute Position",
  // this.fourEncoder.getAbsolutePosition());
  // SmartDashboard.putNumber("Module Four Encoder Non-Absolute Position",
  // this.fourEncoder.getPosition());
  // }

  // /**
  // * Example command factory method.
  // *
  // * @return a command
  // */
  // public CommandBase exampleMethodCommand() {
  // // Inline construction of command goes here.
  // // Subsystem::RunOnce implicitly requires `this` subsystem.
  // return runOnce(
  // () -> {
  // /* one-time action goes here */
  // });
  // }

  // /**
  // * An example method querying a boolean state of the subsystem (for example, a
  // * digital sensor).
  // *
  // * @return value of some boolean subsystem state, such as a digital sensor.
  // */
  // public boolean exampleCondition() {
  // // Query some boolean state, such as a digital sensor.
  // return false;
  // }

  // @Override
  // public void periodic() {
  // // This method will be called once per scheduler run
  // }

  // @Override
  // public void simulationPeriodic() {
  // // This method will be called once per scheduler run during simulation
  // }

  // private enum ButtonModes {
  // TestingMotors,
  // ZeroingTurnEncoders,
  // AlignMotors
  // }
}
