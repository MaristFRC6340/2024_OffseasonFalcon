// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * For Training Purposes
 */

public class TrainingAuto extends SequentialCommandGroup {

  /**
   * Autonomous for training programmings to set paths and Poses
   * @param s_Swerve Swerve Drive Subsystem
   */
  public TrainingAuto(SwerveSubsystem s_Swerve){
      TrajectoryConfig config =
          new TrajectoryConfig(
                  Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                  Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(Constants.Swerve.swerveKinematics);

      /**
       * This is where you will set the Paths
       * Change the values in Points to Pass Through List
       * Robot's Starting Position is the origin at (0, 0)
       * All Units in Meters
       */
      Trajectory exampleTrajectory = 
          TrajectoryGenerator.generateTrajectory(
              // Starting Pose
              new Pose2d(0, 0, new Rotation2d(0)),

              // Points to Pass Through
              List.of(new Translation2d(2,0)),

              // Ending Pose
              new Pose2d(0, 0, new Rotation2d(0)),
          config);

      var thetaController =
          new ProfiledPIDController(
              Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //   SwerveControllerCommand swerveControllerCommand =
    //       new SwerveControllerCommand(
    //           exampleTrajectory,
    //           s_Swerve::getPose,
    //           Constants.Swerve.swerveKinematics,
    //           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //           thetaController,
    //           s_Swerve::setModuleStates,
    //           s_Swerve);


      addCommands(
          new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose()))
          //swerveControllerCommand
      );
  }
}
