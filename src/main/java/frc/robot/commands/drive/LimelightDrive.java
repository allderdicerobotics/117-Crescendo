package frc.robot.commands.drive;

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
        addRequirements(swerve);
    }

    
    @Override
    public void execute(){

        double fwdSpeed = driverController.getLeftY();
        double strafeSpeed = driverController.getLeftX();
        double rotSpeed = driverController.getRightX();

        double translationVal = -MathUtil.applyDeadband(fwdSpeed, Constants.Swerve.stickDeadband);
        double strafeVal = -MathUtil.applyDeadband(strafeSpeed, Constants.Swerve.stickDeadband);

        var result = limelight.getLatestResult();
        
        double rotVal = result.hasTargets() 
        ? -turnPIDController.calculate(result.getBestTarget().getYaw(),0)
        : -MathUtil.applyDeadband(rotSpeed, Constants.Swerve.stickDeadband);        
        
        swerve.drive(
            new Translation2d(translationVal,strafeVal).times(Constants.Swerve.maxSpeed),
            rotVal * Constants.Swerve.maxAngularVelocity,
            true,
            false
        );
    }
}
