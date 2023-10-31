// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveKinematics;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.commands.IntakeLaunch;
import frc.robot.commands.IntakePivot;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.IntakeLaunchSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;;

public class RobotContainer {

  public static RobotContainer instance;

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  // Instance of the Swerve Subsystem
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // Instance of the Controller used to drive the robot
  private CommandXboxController driveController = new CommandXboxController(
      ControllerConstants.DRIVE_CONTROLLER_ID);
  private CommandXboxController intakeController = new CommandXboxController(ControllerConstants.INTAKE_CONTROLLER_ID);

  /**
   * Creates an instance of the robot controller
   */
  public RobotContainer() {

    // sets the default command to being driving swerve
    swerveSubsystem.setDefaultCommand(new SwerveDrive(swerveSubsystem,
        () -> -driveController.getLeftY(),
        () -> -driveController.getLeftX(),
        () -> driveController.getRawAxis(Constants.ControllerConstants.DRIVE_ROT_AXIS),
        () -> !driveController.getHID().getAButton(),
        () -> driveController.getHID().getRightBumper()));

    configureBindings();
  }

  /**
   * Configures the button bindings of the controller
   */
  private void configureBindings() {
    driveController.y().whileTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));

    // intakeController.y()
    // .whileTrue(Commands.parallel(new IntakePivot(IntakeMode.INTAKING),
    // Commands.repeatingSequence(new IntakeLaunch(IntakeMode.INTAKING))));

    //this.intakeController.y().whileTrue(new IntakeLaunch(IntakeMode.INTAKING));
    
    intakeController.y().whileTrue(new IntakePivot(IntakeMode.LOW));
  //   intakeController.y()
  //    .whileTrue(Commands.parallel(new IntakePivot(IntakeMode.INTAKING), new
  //   IntakeLaunch(IntakeMode.INTAKING)));
  //   intakeController.x()
  //       .whileTrue(new IntakeLaunch(IntakeMode.HIGH));
  //   intakeController.a().whileTrue(new IntakeLaunch(IntakeMode.MIDDLE));
  //   intakeController.b().whileTrue(Commands.parallel(new IntakePivot(IntakeMode.LOW),
  //       Commands.sequence(Commands.waitSeconds(1), new IntakeLaunch(IntakeMode.LOW))));
  //   intakeController.rightBumper().whileTrue(Commands.parallel(new IntakePivot(IntakeMode.LONG),
  //       Commands.sequence(Commands.waitSeconds(1), new IntakeLaunch(IntakeMode.LONG))));
  }

  /**
   * @return Command to be run durring autonomous mode
   */
  public Command getAutonomousCommand() {
    // Create trajectory settings

    try {
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(SwerveKinematics.MAX_DRIVE_SPEED_METERS_PER_SECOND,
          SwerveKinematics.MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(SwerveKinematics.driveKinematics);

      // Generate Trajectory
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,
          0, new Rotation2d(0)),
          List.of(new Translation2d(-1,0)),
          new Pose2d(-2, 0, new Rotation2d(0)),
          trajectoryConfig);

      // Define PID controllers for tracking trajectory

      PIDController xController = new PIDController(SwerveKinematics.KP, SwerveKinematics.KI, 0);
      PIDController yController = new PIDController(SwerveKinematics.KP, SwerveKinematics.KI, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(SwerveKinematics.KP,
          SwerveKinematics.KI,
          0,
          SwerveKinematics.AUTO_PID_THETA_CONTROLLER_CONSTRAINTS);

      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      // System.out.println("Deploy" + Filesystem.getDeployDirectory());

      /*
       * // Check if the file exist and is readable
       * String pathFilename = "TestPath";
       * String exportedPathFile = Filesystem.getDeployDirectory() +
       * "/pathplanner/paths/" + pathFilename + ".path";
       * // Placeholder for now
       * PathPlannerPath path = new PathPlannerPath(null, null, null, null, null,
       * null, false);
       * 
       * if (exportedPathFile != null && exportedPathFile != "") {
       * File file = new File(exportedPathFile);
       * if (file != null && file.exists() && file.canRead()) {
       * path = PathPlannerPath.fromPathFile(pathFilename);
       * if (path == null) {
       * System.out.println("Unable to generate PathPlannerPath object");
       * } else {
       * System.out.println("PathPlannerPath object created");
       * }
       * } else {
       * System.out.println("The file [" + exportedPathFile +
       * "] does not exist or is not readable");
       * }
       * } else {
       * System.out.println("The exported path file is either null or empty");
       * }
       * 
       * PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new
       * ChassisSpeeds());
       * // HashMap<String, Command> eventMap = new HashMap<>();
       * 
       * ReplanningConfig replanningConfig = new ReplanningConfig();
       * HolonomicPathFollowerConfig pathConfig = new HolonomicPathFollowerConfig(
       * new PIDConstants(SwerveKinematics.KP, SwerveKinematics.KI, 0),
       * new PIDConstants(SwerveKinematics.KP, SwerveKinematics.KI, 0),
       * SwerveKinematics.MAX_DRIVE_SPEED_METERS_PER_SECOND, 1, replanningConfig);
       * 
       * AutoBuilder.configureHolonomic(swerveSubsystem::getPose,
       * swerveSubsystem::resetOdometry,
       * swerveSubsystem::getChassisSpeeds, swerveSubsystem::setChasisSpeeds,
       * pathConfig, swerveSubsystem);
       * 
       * Command auto = AutoBuilder.followPathWithEvents(path);
       */
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
          swerveSubsystem::getPose,
          SwerveKinematics.driveKinematics, xController, yController,
          thetaController,
          swerveSubsystem::setModuleStates,
          swerveSubsystem);

      // SequentialCommandGroup autoCommand = new SequentialCommandGroup(
      // new InstantCommand(() ->
      // swerveSubsystem.resetOdometry(trajectory.getInitialTargetHolonomicPose())),
      // auto,
      // new InstantCommand(() -> swerveSubsystem.stopModules()),
      // new InstantCommand(() -> System.out.println("Auto Complete")));
      // autoCommand.initialize();
      
      Command autoCommand = Commands.sequence(new InstantCommand(()-> swerveSubsystem.zeroHeading()), swerveControllerCommand, new InstantCommand(()-> swerveSubsystem.stopModules()), new InstantCommand(()-> swerveSubsystem.zeroHeading()));
      return autoCommand;
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    } finally {
      System.out.println("Generated Auto");
    }
    /*
     * return new SequentialCommandGroup(z
     * new InstantCommand(() ->
     * swerveSubsystem.resetOdometry(trajectory.getInitialDifferentialPose())),
     * swerveControllerCommand,
     * new InstantCommand(() -> swerveSubsystem.stopModules()));
     */

  }
}
