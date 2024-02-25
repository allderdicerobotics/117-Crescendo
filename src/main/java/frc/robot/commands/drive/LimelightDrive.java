package frc.robot.commands.drive;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.misc.SquaredSlewRate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class LimelightDrive extends Command {
    private Drive swerve;
    private Vision limelight;
    private PS4Controller driverController;
    private SquaredSlewRate slewRate;
    private PIDController turnPIDController = new PIDController(
        Constants.Swerve.adjustKP,
        Constants.Swerve.adjustKI,
        Constants.Swerve.adjustKD
    );

    public LimelightDrive(Drive swerve, Vision limelight, PS4Controller driverController){
        this.swerve = swerve;
        this.limelight = limelight;
        this.driverController = driverController;
        slewRate = new SquaredSlewRate(0.3, Constants.Swerve.stickDeadband);
        addRequirements(swerve);
    }

    
    @Override
    public void execute(){

        double fwdSpeed = driverController.getLeftY();
        double strafeSpeed = driverController.getLeftX();
        double rotSpeed = driverController.getRightX();

        double translationVal = slewRate.calculate(fwdSpeed);
        double strafeVal = slewRate.calculate(strafeSpeed);
        double rotVal = slewRate.calculate(rotSpeed);
        // double translationVal = -MathUtil.applyDeadband(
        //     Math.copySign(Math.pow(fwdSpeed,2), fwdSpeed)
        //     , Constants.Swerve.stickDeadband);
        // double strafeVal = -MathUtil.applyDeadband(
        //     Math.copySign(Math.pow(strafeSpeed,2), strafeSpeed)
        //     , Constants.Swerve.stickDeadband);

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
                rotVal = turnPIDController.calculate(speakerTarget.getYaw(),5);
            }
            
            
        }
                
        
        swerve.drive(
            new Translation2d(translationVal,strafeVal).times(Constants.Swerve.maxSpeed),
            rotVal * Constants.Swerve.maxAngularVelocity,
            true,
            false
        );
    }
}
