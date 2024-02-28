package frc.robot.commands.drive;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class LimelightDrive extends Command {
    private Drive swerve;
    private Vision limelight;
    private PS4Controller driverController;
    private PIDController turnPIDController = new PIDController(
        Constants.Swerve.adjustKP,
        Constants.Swerve.adjustKI,
        Constants.Swerve.adjustKD
    );

    public LimelightDrive(Drive swerve, Vision limelight, PS4Controller driverController){
        this.swerve = swerve;
        this.limelight = limelight;
        this.driverController = driverController;
        turnPIDController.setTolerance(0.5);
        addRequirements(swerve);
    }

    
    @Override
    public void execute(){

        double fwdSpeed = driverController.getLeftY();
        double strafeSpeed = driverController.getLeftX();
        double rotSpeed = driverController.getRightX();

        double translationVal = -MathUtil.applyDeadband(
            Math.copySign(Math.pow(fwdSpeed,2), fwdSpeed)
            , Constants.Swerve.stickDeadband);
        double strafeVal = -MathUtil.applyDeadband(
            Math.copySign(Math.pow(strafeSpeed,2), strafeSpeed)
            , Constants.Swerve.stickDeadband);
        double rotVal = -MathUtil.applyDeadband(
            Math.copySign(Math.pow(rotSpeed,2), rotSpeed)
            , Constants.Swerve.stickDeadband);;

        var result = limelight.getLatestResult();
        PhotonTrackedTarget speakerTarget = null;
        if (result.hasTargets()){
            var hasDesiredTarget = false;
            for (PhotonTrackedTarget target : result.getTargets()){
                if (target.getFiducialId() == 4 || target.getFiducialId() == 7){
                    speakerTarget = target;
                    hasDesiredTarget = true;
                    break;
                }
            }
            if (hasDesiredTarget && speakerTarget != null){
                System.out.println(speakerTarget.getYaw());
                // System.out.println(speakerTarget.getBestCameraToTarget().getX());
                rotVal = turnPIDController.calculate(speakerTarget.getYaw(),0);
            }
            
            
        }
                
        
        swerve.drive(
            new Translation2d(translationVal,strafeVal).times(Constants.Swerve.maxSpeed),
            rotVal * Constants.Swerve.maxAngularVelocity
        );
    }
    // @Override
    // public boolean isFinished(){
    //     return turnPIDController.atSetpoint();
    // }
    @Override
    public void end(boolean interrupted){
        
    }
}
