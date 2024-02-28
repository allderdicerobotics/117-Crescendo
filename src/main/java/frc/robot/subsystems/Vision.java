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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Vision extends SubsystemBase {
    private String cameraName;
    private PhotonCamera limelight;
    private PhotonPoseEstimator visionEstimator;
    private AprilTagFieldLayout layout;
    private Alliance alliance;
    

    public Vision(String cameraName, AprilTagFieldLayout layout, Alliance alliance) {
        this.cameraName = cameraName;
        this.layout = layout;
        this.alliance = alliance;

        limelight = new PhotonCamera(cameraName);
        // layout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
        //         : OriginPosition.kRedAllianceWallRightSide);

        // use PhotonLib's PoseEstimator on AprilTags
        visionEstimator = new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            limelight,
            Constants.Vision.robotToCam
        );
        visionEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // Constants.Logging.poseEstimationTab.addDouble(
        //     "Limelight Distance", this::getTagDistance
        // );
       
    }

    public PhotonPipelineResult getLatestResult(){
        return limelight.getLatestResult();
    }

    public Alliance getAlliance(){
        return this.alliance;
    }

    public Pose2d getTagPose(int fiducialID){
        return this.layout.getTagPose(fiducialID).get().toPose2d();
    }

    public double getTagDistance(){
        var latestResult = getLatestResult();
        if (latestResult.hasTargets()){
            var bestTarget = latestResult.getBestTarget();
            var camToTarget = bestTarget.getBestCameraToTarget();
            return camToTarget.getX();
        }
        return -1;   
    }
    public Optional<EstimatedRobotPose> updateVision(Pose2d prevEstimatedPose) {
        PhotonPipelineResult pipelineResult = getLatestResult();
        visionEstimator.setReferencePose(prevEstimatedPose);
        return visionEstimator.update(pipelineResult);
    }

    public Matrix<N3,N1> getStdDevs(Pose2d estPose){
        var estStdDevs = VecBuilder.fill(4, 4, 8);
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = visionEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
      
    }

    
}

