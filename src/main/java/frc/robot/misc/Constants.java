package frc.robot.misc;

import com.revrobotics.CANSparkBase.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.sensors.ThriftyEncoder;
import swervelib.SwerveDrive;

public final class Constants {
    public static final class Swerve {
        public static final double stickDeadband = 0.3;

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final double trackWidth = Units.inchesToMeters(18); //TODO: Get the accurate measurement
        public static final double wheelBase = Units.inchesToMeters(18);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double turnGearRatio = (11.8 / 1.0);//(150.0 / 7.0);

        public static final SwerveDriveKinematics swerveKinematics = 
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            );
            
        // public static final SwerveDriveKinematics swerveKinematics =
        //     new SwerveDriveKinematics(
        //         new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        //         new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        //         new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        //         new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double motorVoltage = 12.0;
        public static final double thriftyMaxVoltage = 4.8;
        
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 40;
        
        /* Angle Motor PID Values */
        public static final double angleKP = 7.5; //1.6 on our robot
        public static final double angleKI = 0;//1.8 on our robot
        public static final double angleKD = 0;
        public static final double angleKFF = 0;

        /* Angle Motor Characterization Values */
        public static final double angleKS = 0;
        public static final double angleKV = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.001; //0.00005 on our robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0001;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
            (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

        /* Turn Motor Conversion Factors */
        public static final double turnConversionPositionFactor = 
            (2 * Math.PI) / turnGearRatio;
        public static final double turnConversionVelocityFactor = 
            turnConversionPositionFactor / 60.0;
            
        /* Turn PID MinMax Input Values */

        public static final double turnPIDMinInput = 0;
        public static final double turnPIDMaxInput = 2 * Math.PI;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5 / 2; // meters per second
        public static final double maxAccel = 250.0;
        public static final double maxAngularVelocity =  4 * Math.PI ;// 11.5; // Math.PI on our robot
        public static final double maxAngularAccel = Math.PI*4;

        /* Neutral Modes */
        public static final IdleMode angleIdleMode = IdleMode.kBrake;
        public static final IdleMode driveIdleMode = IdleMode.kBrake;

        public static final double minOutputRangeTurn = -1;
        public static final double maxOutputRangeTurn = 1;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8; // 6;
            public static final int thriftyEncoderID = 2;
            
            public static final double angleOffset = -64.2; // 
            public static final ThriftyEncoder thriftyEncoder = new ThriftyEncoder(new AnalogInput(thriftyEncoderID)).shiftDegs(angleOffset);
            
            
            public static final SwerveModule module =
                new SwerveModule(
                    driveMotorID, 
                    angleMotorID, 
                    thriftyEncoder, 
                    "FL",
                    false,
                    true
                );
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int thriftyEncoderID = 1;
            public static final double angleOffset = 28.2; // 
            public static final ThriftyEncoder thriftyEncoder = new ThriftyEncoder(new AnalogInput(thriftyEncoderID)).shiftDegs(angleOffset);

            public static final SwerveModule module =
                new SwerveModule(
                    driveMotorID, 
                    angleMotorID, 
                    thriftyEncoder, 
                    "FR",
                    false,
                    true
                );
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 6;//8;
            public static final int thriftyEncoderID = 3;
            public static final double angleOffset = -56.7; //
            public static final ThriftyEncoder thriftyEncoder = new ThriftyEncoder(new AnalogInput(thriftyEncoderID)).shiftDegs(angleOffset);

            public static final SwerveModule module =
                new SwerveModule(
                    driveMotorID, 
                    angleMotorID, 
                    thriftyEncoder, 
                    "BL",
                    false,
                    true
                );
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int thriftyEncoderID = 0;
            public static final double angleOffset = -92.3;
            public static final ThriftyEncoder thriftyEncoder = new ThriftyEncoder(new AnalogInput(thriftyEncoderID)).shiftDegs(angleOffset);

            public static final SwerveModule module =
                new SwerveModule(
                    driveMotorID, 
                    angleMotorID, 
                    thriftyEncoder, 
                    "BR",
                    false,
                    true
                );
        }
    }
    public static final class Intake {
        public static final int motorID = -1; // TODO: Get the ID
        

    }
    public static final class Shooter {
        public static final int motorID = -1; // TODO: Get the ID
    }
    public static final class Tower {
        public static final int motorID = -1; // TODO: Get the ID
    }

    public static final class Climber {
        public static final int leftMotorID = -1; // TODO: Get the ID
        public static final int rightMotorID = -1; // TODO: Get the ID
    }

    public static final class AutoConstants { //TODO: Adjust these when we make our own auto
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}