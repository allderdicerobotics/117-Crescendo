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
    private SwerveDrivePoseEstimator m_swervePoseEstimator;
    private Drive m_swerve;
    private Vision m_limelight;
    private NavX m_NavX;
    private boolean m_usingVision;
    public PoseEstimator(Drive swerve, Vision limelight, NavX navx, boolean usingVision) {
        // TODO: Add limelight code
        m_swerve = swerve;
        m_limelight = limelight;
        m_NavX = navx;
        m_usingVision = usingVision;

        m_swervePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            m_NavX.getAngle(),
            m_swerve.getModulePositions(),
            new Pose2d());
        SmartDashboard.putData("Field", field2d);
    }

    public void zeroAngle() {
        m_NavX.zeroYaw();
        m_swervePoseEstimator.resetPosition(
            m_NavX.getAngle(),
            m_swerve.getModulePositions(),
            new Pose2d(getPose().getTranslation(), new Rotation2d())
        );
    }

    public Pose2d getPose() {
        return m_swervePoseEstimator.getEstimatedPosition();
        
    }

    public void updatePose(){
        if (m_usingVision){

        }
        m_swervePoseEstimator.update(m_NavX.getAngle(), m_swerve.getModulePositions());
    }

    
    public void resetPose(Pose2d pose) {
        m_swervePoseEstimator.resetPosition(m_NavX.getAngle(),m_swerve.getModulePositions(),pose);
    }

    @Override
    public void periodic(){
        DriverStation.refreshData();
        field2d.setRobotPose(getPose());
        SmartDashboard.putData(field2d);
        SmartDashboard.putNumber("NavX angle",m_NavX.getAngle().getDegrees());
    }
}
