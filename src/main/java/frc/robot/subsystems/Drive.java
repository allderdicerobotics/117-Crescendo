package frc.robot.subsystems;


import com.gos.lib.swerve.SwerveDrivePublisher;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.misc.Constants;
import frc.robot.misc.Constants.Swerve.*;
import frc.robot.sensors.NavX;

public class Drive extends SubsystemBase {
	// Robot swerve modules

	// Odometry class for tracking robot pose
	private SwerveDrivePoseEstimator swerveOdometry;
	private NavX navx = new NavX();
	private SwerveDrivePublisher publisher;
	private final Field2d field;
	/** Creates a new DriveSubsystem. */
	public Drive() {
		resetEncoders();
		field = new Field2d();
		publisher = new SwerveDrivePublisher();
		
		Timer.delay(1);
		swerveOdometry = new SwerveDrivePoseEstimator(
			Constants.Swerve.swerveKinematics, 
			navx.getAngle(), 
			getModulePositions(), 
			new Pose2d()
		);
		AutoBuilder.configureHolonomic(
			this::getPose, 
			this::resetPose, 
			this::getRobotRelativeSpeeds, 
			this::driveRobotRelative, 
			new HolonomicPathFollowerConfig(
				new PIDConstants(5.0),
				new PIDConstants(5.0),
				Constants.Swerve.maxSpeed,
				Units.inchesToMeters(Constants.Swerve.wheelBase),
				new ReplanningConfig()
			), 
			() -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
				}, 
				this
		);
	}

	@Override
	public void periodic() {
		/* Update Odometry */
		swerveOdometry.update(navx.getAngle(), getModulePositions());
		field.setRobotPose(getPose());
		SmartDashboard.putData("Field", field);
		publisher.setMeasuredStates(getModuleStates());
		SmartDashboard.putNumber("NavX Angle",navx.getAngle().getDegrees());
	}

	// // Update the odometry in the periodic block
	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop) {
		var swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
			fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
				translation.getX(),
				translation.getY(),
				rotation,
				navx.getAngle()
			) : new ChassisSpeeds(
				translation.getX(),
				translation.getY(),
				rotation
			)
		);
		
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,Constants.Swerve.maxSpeed);
		publisher.setDesiredStates(swerveModuleStates);
		Mod0.module.setDesiredState(swerveModuleStates[0], openLoop);
		Mod1.module.setDesiredState(swerveModuleStates[1], openLoop);
		Mod2.module.setDesiredState(swerveModuleStates[2], openLoop);
		Mod3.module.setDesiredState(swerveModuleStates[3], openLoop);

	}
	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				desiredStates, Constants.Swerve.maxSpeed);
		Mod0.module.setDesiredState(desiredStates[0], false);
		Mod1.module.setDesiredState(desiredStates[1], false);
		Mod2.module.setDesiredState(desiredStates[2], false);
		Mod3.module.setDesiredState(desiredStates[3], false);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		Mod0.module.reset();
		Mod1.module.reset();
		Mod2.module.reset();
		Mod3.module.reset();
	}

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
			Mod0.module.getPosition(),
			Mod1.module.getPosition(),
			Mod2.module.getPosition(),
			Mod3.module.getPosition()
		};
	}	
	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
			Mod0.module.getState(),
			Mod1.module.getState(),
			Mod2.module.getState(),
			Mod3.module.getState()
		};
	}	
	public Pose2d getPose(){
		return swerveOdometry.getEstimatedPosition();
	}
	public void resetPose(Pose2d pose){
		swerveOdometry.resetPosition(navx.getAngle(), getModulePositions(), pose);
	}
	public void resetOrientation(){
		navx.zeroYaw();
	}

	private ChassisSpeeds getRobotRelativeSpeeds(){
		SwerveModuleState[] modStates = getModuleStates();
		return Constants.Swerve.swerveKinematics.toChassisSpeeds(
			modStates[0],
			modStates[1],
			modStates[2],
			modStates[3]
		);
	}
	
	public void driveRobotRelative(ChassisSpeeds speeds){
		Translation2d translate = new Translation2d(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond);
		drive(translate,speeds.omegaRadiansPerSecond,false,false);
	}
	
}
