package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Vision extends SubsystemBase {
    // TODO: Make code when limelight gets here
    private String cameraName = Constants.Vision.cameraName;
    private PhotonCamera limelight;
    private AprilTagFieldLayout layout;
    private double latestTimeStamp = 0;

    public Vision() {
        limelight = new PhotonCamera(cameraName);

        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        var alliance = DriverStation.getAlliance();
        layout.setOrigin(alliance.get() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide);
    }

    public Pose2d updateVision() {
        PhotonPipelineResult pipelineResult = limelight.getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();
        if (pipelineResult.hasTargets() && resultTimestamp != latestTimeStamp) {
            latestTimeStamp = resultTimestamp;
            var target = pipelineResult.getBestTarget();
            var tagID = target.getFiducialId();
            Optional<Pose3d> tagPose = (layout.getTagPose(tagID));
            if (target.getPoseAmbiguity() <= 0.2 && tagID >= 0 && tagPose.isPresent()) {
                
            }

        }
    }
}
