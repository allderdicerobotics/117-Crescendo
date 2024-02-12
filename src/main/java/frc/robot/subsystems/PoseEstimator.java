package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import frc.robot.sensors.NavX;

public class PoseEstimator extends SubsystemBase {
    private Field2d field2d = new Field2d();
    private SwerveDrivePoseEstimator poseEstimator;
    private Drive swerve;
    private Vision limelight;
    private NavX navx;
    private boolean usingVision;

    public PoseEstimator(Drive swerve, Vision limelight, NavX navx, boolean usingVision) {

        this.swerve = swerve;
        this.limelight = limelight;
        this.navx = navx;
        this.usingVision = usingVision;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                navx.getAngle(),
                swerve.getModulePositions(),
                new Pose2d());
        SmartDashboard.putData("Field", field2d);
    }

    public void zeroAngle() {
        navx.zeroYaw();
        poseEstimator.resetPosition(
                navx.getAngle(),
                swerve.getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();

    }

    public void updatePose() {
        poseEstimator.update(navx.getAngle(), swerve.getModulePositions());
        if (usingVision) {
            var visionEst = limelight.updateVision();

            visionEst.ifPresent(
                    est -> {
                        var estPose = est.estimatedPose.toPose2d();

                        poseEstimator.addVisionMeasurement(estPose, est.timestampSeconds); // TODO: Figure out
                                                                                           // if the std devs
                                                                                           // function is useful
                    });
        }

    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(navx.getAngle(), swerve.getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        DriverStation.refreshData();
        field2d.setRobotPose(getPose());
        SmartDashboard.putData(field2d);
        SmartDashboard.putNumber("NavX angle", navx.getAngle().getDegrees());
    }
}
