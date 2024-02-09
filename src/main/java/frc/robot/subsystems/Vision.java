package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.misc.Constants;

public class Vision extends SubsystemBase{
    //TODO: Make code when limelight gets here
    private String limelightName = Constants.Vision.limelightName;
    public Vision(){

    }
    public Pose2d getBotPose(){
        return LimelightHelpers.getBotPose2d(limelightName);
    }
    public double getLatency(){
        return LimelightHelpers.getLatency_Capture(limelightName);
    }
    public double getX(){
        return LimelightHelpers.getTX(limelightName);
    }
    public double getY(){
        return LimelightHelpers.getTY(limelightName);
    }
}
