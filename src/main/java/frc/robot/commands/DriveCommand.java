package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class DriveCommand extends Command{
    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier leftX;
    private DoubleSupplier leftY;
    private DoubleSupplier rightX;
    public DriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX){
        this.swerveSubsystem = swerveSubsystem;
        this.leftX=leftX;
        this.leftY=leftY;
        this.rightX=rightX;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // ChassisSpeeds desiredSpeeds = swerveSubsystem.getTargetSpeeds(leftX.getAsDouble(), leftY.getAsDouble(), rightX.getAsDouble())
        // Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

        swerveSubsystem.drive(new Translation2d(leftX.getAsDouble(), leftY.getAsDouble()), rightX.getAsDouble()*Constants.Swerve.maxAngularVelocity, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    }
    

    @Override
    public boolean isFinished(){
        return false;
    }
}
