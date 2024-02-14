package frc.robot.misc;

import com.revrobotics.CANSparkBase.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.ThriftyEncoder;

public final class Constants {
    public static final double globalVoltageCompensation = 12;

    public static final class Swerve {
        public static final double stickDeadband = 0.3;

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final double trackWidth = Units.inchesToMeters(21);
        public static final double wheelBase = Units.inchesToMeters(21);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.54 / 1.0); // 6.54:1
        public static final double turnGearRatio = (11.8 / 1.0); // unused (needed for Position PID control)

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 40;

        /* Angle Motor PID Values */
        public static final double angleKP = 5.0; // 1.6 on our robot
        public static final double angleKI = 0;// 1.8 on our robot
        public static final double angleKD = 0;
        public static final double angleKFF = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.001; // 0.00005 on our robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0001;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

        /* Turn Motor Conversion Factors */
        public static final double turnConversionPositionFactor = (2.0 * Math.PI) / turnGearRatio;
        public static final double turnConversionVelocityFactor = turnConversionPositionFactor / 60.0;

        /* Turn PID MinMax Input Values */

        public static final double turnPIDMinInput = 0;
        public static final double turnPIDMaxInput = 2.0 * Math.PI;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5 / 2.0; // meters per second
        public static final double maxAccel = 250.0;
        public static final double maxAngularVelocity = 4.0 * Math.PI;// 11.5; // Math.PI on our robot
        public static final double maxAngularAccel = Math.PI * 4.0;

        /* Neutral Modes */
        public static final IdleMode angleIdleMode = IdleMode.kBrake;
        public static final IdleMode driveIdleMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8; // 6;
            public static final int thriftyEncoderID = 2;

            public static final double angleOffset = -65.7; //
            public static final ThriftyEncoder thriftyEncoder = new ThriftyEncoder(new AnalogInput(thriftyEncoderID))
                    .shiftDegs(angleOffset);

            public static final SwerveModule module = new SwerveModule(
                    driveMotorID,
                    angleMotorID,
                    thriftyEncoder,
                    "FL",
                    true,
                    true);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int thriftyEncoderID = 1;
            public static final double angleOffset = 27.4; //
            public static final ThriftyEncoder thriftyEncoder = new ThriftyEncoder(new AnalogInput(thriftyEncoderID))
                    .shiftDegs(angleOffset);

            public static final SwerveModule module = new SwerveModule(
                    driveMotorID,
                    angleMotorID,
                    thriftyEncoder,
                    "FR",
                    true,
                    true);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 6;// 8;
            public static final int thriftyEncoderID = 3;
            public static final double angleOffset = -58.7; //
            public static final ThriftyEncoder thriftyEncoder = new ThriftyEncoder(new AnalogInput(thriftyEncoderID))
                    .shiftDegs(angleOffset);

            public static final SwerveModule module = new SwerveModule(
                    driveMotorID,
                    angleMotorID,
                    thriftyEncoder,
                    "BL",
                    true,
                    true);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int thriftyEncoderID = 0;
            public static final double angleOffset = -90.4;
            public static final ThriftyEncoder thriftyEncoder = new ThriftyEncoder(new AnalogInput(thriftyEncoderID))
                    .shiftDegs(angleOffset);

            public static final SwerveModule module = new SwerveModule(
                    driveMotorID,
                    angleMotorID,
                    thriftyEncoder,
                    "BR",
                    true,
                    true);
        }
    }

    public static final class Intake {
        public static final int motorID = 10; // TODO: Get the ID
        public static final int currentLimit = 20;

    }

    public static final class Operator {

        public static final int driverControllerPort = 0;
        public static final int buttonBoardPort = 2;
        public static PS4Controller driverController = new PS4Controller(driverControllerPort);
        public static Joystick operatorController = new Joystick(buttonBoardPort);

        public static final int translationAxis = 1;
        public static final int strafeAxis = 0;
        public static final int rotAxis = 2;

        private static final int intakeCmdButton = 1;
        public static Trigger intakeTrigger = new JoystickButton(driverController, intakeCmdButton);

        private static final int shooterCmdButton = 2;
        public static Trigger shooterTrigger = new JoystickButton(driverController, shooterCmdButton);

        private static final int towerCmdButton = 3;
        public static Trigger towerTrigger = new JoystickButton(driverController, towerCmdButton);

        private static final int climberToggleButton = 4;
        public static Trigger climberTrigger = new JoystickButton(driverController, climberToggleButton);

        private static final int resetGyroButton = 4;
        public static Trigger resetGyroTrigger = new JoystickButton(driverController, resetGyroButton);

        // private static final int 
    }

    public static final class Indexer {
        public static final int motorID = 11;
        public static final int currentLimit = 20;
        public static final int sensorID = 0;
        public static final double threshDist = 115; // cm to mm
    }

    public static final class Shooter {
        public static final int topID = 12;
        public static final int bottomID = 13;

        public static final int shooterCurrentLimit = 20;

        public static final double shooterKP = 1.0;
        public static final double shooterKI = 0.0;
        public static final double shooterKD = 0.0;

        public static final double shooterKFF = 1.0;

        public static final double maxNegPower = -0.5;
        public static final double maxPosPower = 1.0;
        public static final double speakerRPM = 5000;
        public static final double apmRPM = 800;
        public static final double threshRPM = 10;
    }

    public static final class Tower {
        public static final int motorID = 14; // TODO: Get the ID
        public static final double gearReduction = ((64.0 / 22.0) * 25.0) / 1.0;
        public static final double pivotKP = 0.0;
        public static final double threshAngle = 0.1; // deg
        public static final int towerCurrentLimit = 20;
        public static final double maxNegPower = -0.5;
        public static final double maxPosPower = 1;
        public static final int minAngle = 0;
        public static final int maxAngle = 50; //TODO: GET A CORRECT VALUE

    }

    public static final class Climber {
        public static final int leftMotorID = 15; // TODO: Get the ID
        public static final int rightMotorID = 16; // TODO: Get the ID
        public static final int currentLimit = 20;
        public static final int homeCurrent = 10;
        public static final double legalMax = (175.0 / 12.0);
        public static final double homingSpeed = -0.4;
        public static final double positionConversionFactor = (12.0 / 1.0);
        public static final double velocityConversionFactor = positionConversionFactor / 60.0;
        public static final double climberKP = 0.0;
        public static final double climberKI = 0.0;
        public static final double climberKD = 0.0;

    }

    public static final class Vision {

        public static final String cameraName = "limelight";

        public static final int blueSpeakerTagID = 7;
        public static final int redSpeakerTagID = 4;

        public static final Transform3d robotToCam = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(2),
                        Units.inchesToMeters(3),
                        Units.inchesToMeters(10)),
                new Rotation3d());
    }
}