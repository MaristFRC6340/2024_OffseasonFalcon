package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

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

public class GamePathExampleMultipleTrajectories extends SequentialCommandGroup {
    public GamePathExampleMultipleTrajectories(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(3, 0, new Rotation2d(0)),
        //         config);

        Trajectory traj1 = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(.05,.3),
                new Translation2d(.052, .4)),
                new Pose2d(-0.1, 0.6, new Rotation2d(0)),
            config);

            Trajectory traj2 = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(-0.25, 0.6, new Rotation2d(0)),
                List.of(
                new Translation2d(-2.4,.7),
                new Translation2d(-3.6, .65)),
                new Pose2d(-4.7, 0.6, new Rotation2d(Math.toRadians(180))),
            config);

            Trajectory traj3 = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(-4.83, 0.6, new Rotation2d(Math.toRadians(180))),
                List.of(new Translation2d(-3.6,.5),
                new Translation2d(-2.4,.7),
                new Translation2d(0, .6),
                new Translation2d(0,.3)),
                new Pose2d(0,0, new Rotation2d(0)),
            config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand traj1Command =
            new SwerveControllerCommand(
                traj1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

            SwerveControllerCommand traj2Command =
            new SwerveControllerCommand(
                traj2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
            
                SwerveControllerCommand traj3Command =
            new SwerveControllerCommand(
                traj3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);       

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(traj1.getInitialPose())),
            traj1Command,
            traj2Command,
            traj3Command

        );
    }
}