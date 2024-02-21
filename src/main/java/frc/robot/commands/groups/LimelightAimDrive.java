package frc.robot.commands.groups;


import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.LimelightDrive;
import frc.robot.commands.tower.LimelightAimTower;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Tower;

public class LimelightAimDrive extends ParallelCommandGroup{
    private Drive swerve;
    private Vision limelight;
    private PoseEstimator poseEstimator;
    private Tower tower;
    private PS4Controller ps4Controller;
    
    public LimelightAimDrive(Drive swerve, Vision limelight, PoseEstimator poseEstimator, Tower tower, PS4Controller ps4Controller){
        addCommands(
            new LimelightAimTower(tower, poseEstimator),
            new LimelightDrive(swerve, limelight, ps4Controller)
        );
    }
}
