package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.misc.Constants;
import frc.robot.subsystems.Operator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AdjustShooter extends Command {
    private Vision limelight;
    private Shooter shooter;
    private boolean finished;

    public AdjustShooter(Vision limelight, Shooter shooter, Operator OI){
        this.limelight = limelight;
        this.shooter = shooter;
        
    }

    @Override
    public void execute(){
        finished = false;
        
        
        // while (!shooter.atAngle()){
        //     shooter.setAngleShooter(interpolatedAngle);
        // }
        /* while (abs(shooterArmAngle-interpTable(distance)) > threshold) && !driverShootButton
            adjustShooterArm(interTable(distance))

            once finished shoot piece 
        shoot the piece  
        */
        finished = true;
    }
    @Override
    public boolean isFinished(){
        return finished;
    }
    // private double backupInterpolation(){
    //     double distance = 
    //         (Constants.Vision.aprilTagElevation - Constants.Vision.limelightElevation)
    //         /
    //         Math.tan(
    //             Units.degreesToRadians(
    //                 Constants.Vision.limelightAngle + limelight.getY()
    //                 )
    //         );
    //     return shooter.interpolateAngle(distance); 
    // }
}
