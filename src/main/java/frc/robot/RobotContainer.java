// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(Constants.IOConstants.kDriverPort);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  //private final  zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

  /* Subsystems */
  private final SwerveSubsystem s_Swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    //s_Swerve.setDefaultCommand(new PhotonFollow(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    s_Swerve.setDefaultCommand(s_Swerve.driveCommand(
      () -> MathUtil.applyDeadband(driver.getLeftY(), .1),
      () -> MathUtil.applyDeadband(driver.getLeftX(), .1),
      () -> -driver.getRightX(),
      () -> -driver.getRightY()));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new GamePathExampleMultipleTrajectories(s_Swerve);
    //return new RNGAuto(s_Swerve);
  }

  /**
   * This will be another Autonomous Routine for Training
   * programmers
   * 
   * Added by michaudc
   */
  public Command getAutoTrainingCommand() {
    return new TrainingAuto(s_Swerve);
  }
}
