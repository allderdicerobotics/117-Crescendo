package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Vision extends SubsystemBase {
    
    private PhotonCamera limelight;
    private PhotonPoseEstimator visionEstimator;
    private AprilTagFieldLayout layout;

    public Vision() {
        limelight = new PhotonCamera(Constants.Vision.cameraName);
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        var alliance = DriverStation.getAlliance();
        layout.setOrigin(alliance.get() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide);
        visionEstimator = new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            limelight,
            Constants.Vision.robotToCam
        );
        visionEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
       
    }

    public PhotonPipelineResult getLatestResult(){
        return limelight.getLatestResult();
    }
    public Optional<EstimatedRobotPose> updateVision() {
        PhotonPipelineResult pipelineResult = getLatestResult();
        // if (pipelineResult.getTimestampSeconds())
        // if (pipelineResult.hasTargets()){
        //     System.out.println(pipelineResult.getBestTarget().getFiducialId());
        // }
        return visionEstimator.update();
    }

    public Matrix<N3,N1> getStdDevs(Pose2d estPose){
        var singleTagStdDevs = VecBuilder.fill(4,4,8);
        var multiTagStdDevs = VecBuilder.fill(0.5,0.5,1);
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var target : targets){
            var tagPose = visionEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += 
                tagPose.get().toPose2d().getTranslation().getDistance(estPose.getTranslation());

        }
        if (numTags == 0) return singleTagStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) return multiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         
        return singleTagStdDevs.times(1 + (avgDist * avgDist / 30));
      
    }
    
}

