package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class PhotonFollow extends Command {
    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveSubsystem s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    // todo photon vision imports
    // private PhotonCamera camera = new PhotonCamera("Live!_Cam_Chat_HD_VF0790");

    /**
     * Driver control
     */
    public PhotonFollow(SwerveSubsystem s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        
        // todo photon vision code
        /*
        var result = camera.getLatestResult();

        boolean hasTargets = result.hasTargets();
        double yaw = 0;
        double error = 0;

        if (hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            error = target.getYaw();
            System.out.println(error);
            yaw = target.getYaw();
        }
        */

        double yAxis = 0;
        double xAxis = 0;
        double rAxis = -controller.getRawAxis(rotationAxis) * Constants.speedControl;
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        //s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

        // TODO: add gyro reset button
        if(controller.getRawButtonPressed(9)){
            s_Swerve.zeroGyro();
        }
    }
    
}
