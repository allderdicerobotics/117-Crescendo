package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.misc.Constants;
import frc.robot.misc.Constants.Swerve;
import frc.robot.misc.Constants.Swerve.*;
import frc.robot.sensors.NavX;

public class DriveSubsystem extends SubsystemBase {
	// Robot swerve modules

	// Odometry class for tracking robot pose
	private NavX navx = new NavX();
	
	private boolean openLoop;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		resetEncoders();
		Timer.delay(1);
	}

	@Override
	public void periodic() {
		/* Update Odometry */
		// swerveOdometry.update(navx.getAngle(), getModulePositions());
		
		SmartDashboard.putNumber("NavX Angle",navx.getAngle().getDegrees());

		// Mod0.module.setModuleIdleMode(Constants.Swerve.driveIdleMode,Constants.Swerve.angleIdleMode);
		// Mod1.module.setModuleIdleMode(Constants.Swerve.driveIdleMode,Constants.Swerve.angleIdleMode);
		// Mod2.module.setModuleIdleMode(Constants.Swerve.driveIdleMode,Constants.Swerve.angleIdleMode);
		// Mod3.module.setModuleIdleMode(Constants.Swerve.driveIdleMode,Constants.Swerve.angleIdleMode);
		// field2d.setRobotPose(swerveOdometry.getEstimatedPosition());
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
		this.openLoop = openLoop;
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
		Mod0.module.setDesiredState(desiredStates[0], true);
		Mod1.module.setDesiredState(desiredStates[1], true);
		Mod2.module.setDesiredState(desiredStates[2], true);
		Mod3.module.setDesiredState(desiredStates[3], true);
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
}
