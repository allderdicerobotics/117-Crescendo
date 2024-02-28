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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.ThriftyEncoder;

public final class Constants {
    public static final double globalVoltageCompensation = 10;

    public static final class Swerve {
        public static final double stickDeadband = 0.2;

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
        public static final double angleKP = 5; // 1.6 on our robot
        public static final double angleKI = 0;// 1.8 on our robot
        public static final double angleKD = 0;


        /* Limelight Aim PID Values */
        public static final double adjustKP = 0.01;
        public static final double adjustKI = 0.00;
        public static final double adjustKD = 0.0;
        /* Drive Motor PID Values */
        public static final double driveKP = 0.001; // 0.00005 on our robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.25;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 1.0;
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
        public static final double maxSpeed = 4.5 * 0.75; // meters per second
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
            public static final int thriftyEncoderID = 3;

            public static final double angleOffset = -150.45;//-32.3;//-150.5; //-65.7; //
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
            public static final double angleOffset = 2.1;//176.1;//-3.1; //27.4; //
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
            public static final int thriftyEncoderID = 2;
            public static final double angleOffset = -151.1;//-27.2;//210; //-152.3; //-58.7; //
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
            public static final double angleOffset = 115;//-114.3;//115.4; // -90.4;
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

        // Driver Button Bindings

        private static final int sourceIntakeButton = 1;
        public static final Trigger sourceIntakeTrigger = new JoystickButton(driverController, sourceIntakeButton);

        private static final int intakeButton = 7;
        public static Trigger intakeTrigger = new JoystickButton(driverController, intakeButton);

        private static final int resetGyroButton = 4;
        public static Trigger resetGyroTrigger = new JoystickButton(driverController, resetGyroButton);

        private static final int shootAmpButton = 5;
        public static Trigger shootAmpTrigger = new JoystickButton(driverController, shootAmpButton);

        private static final int shootSpeakerButton = 8;
        public static Trigger shootSpeakerTrigger = new JoystickButton(driverController, shootSpeakerButton);

        private static final int toggleAimButton = 14;
        public static Trigger toggleAimTrigger = new JoystickButton(driverController, toggleAimButton);
        
        private static final int reverseIntakeButton = 6;
        public static Trigger reverseIntakeTrigger = new JoystickButton(driverController, reverseIntakeButton);
        

        // Operator Button Bindings

        private static final int towerZeroButton = 1;
        public static Trigger towerZeroTrigger = new JoystickButton(operatorController, towerZeroButton);

        private static final int towerUpButton = 2;
        public static Trigger towerUpTrigger = new JoystickButton(operatorController, towerUpButton);

        private static final int towerDownButton = 3;
        public static Trigger towerDownTrigger = new JoystickButton(operatorController, towerDownButton);

        private static final int farShotButton = 4;
        public static Trigger farShotTrigger = new JoystickButton(operatorController, farShotButton);
        
        private static final int leftClimberUpButton = 5;
        public static Trigger leftClimberUpTrigger = new JoystickButton(operatorController, leftClimberUpButton);

        private static final int rightClimberUpButton = 6;
        public static Trigger rightClimberUpTrigger = new JoystickButton(operatorController, rightClimberUpButton);

        private static final int shooterRampButton = 7;
        public static Trigger shooterRampTrigger = new JoystickButton(operatorController, shooterRampButton);

        // private static final int towerAmpPositionButton = 5; 
        // public static Trigger towerAmpPositionTrigger = new JoystickButton(operatorController, towerAmpPositionButton);

        private static final int midShotButton = 8;
        public static Trigger midShotTrigger = new JoystickButton(operatorController, midShotButton);

        private static final int leftClimberDownButton = 9;
        public static Trigger leftClimberDownTrigger = new JoystickButton(operatorController, leftClimberDownButton);

        private static final int rightClimberDownButton = 10;
        public static Trigger rightClimberDownTrigger = new JoystickButton(operatorController, rightClimberDownButton);

        private static final int ampTowerButton = 11;
        public static Trigger ampTowerTrigger = new JoystickButton(operatorController, ampTowerButton);

        private static final int speakerTowerButton = 12;
        public static Trigger speakerTowerTrigger = new JoystickButton(operatorController, speakerTowerButton);


        

        // private static final int climberToggleButton = 12;
        // public static Trigger climberTrigger = new JoystickButton(driverController, climberToggleButton);


        // private static final int 
    }

    public static final class Indexer {
        public static final int motorID = 17;
        public static final int invMotorID = 21;
        public static final int currentLimit = 20;
        public static final int sensorID = 0;
        public static final double threshDist = 115; // cm to mm
        public static final double gearReduction = (52.0 / 14.0);
    }

    public static final class Shooter {
        public static final int topID = 16;
        public static final int bottomID = 12;

        public static final int shooterCurrentLimit = 20;

        public static final double shooterKP = 0.35;
        public static final double shooterKI = 0.0;
        public static final double shooterKD = 0.0;

        public static final double shooterKFF = 0.75;

        public static final double maxNegPower = -0.5;
        public static final double maxPosPower = 1.0;
        public static final double speakerRPM = 5000;
        public static final double apmRPM = 400;
        public static final double threshRPM = 5;
    }

    public static final class Tower {
        public static final int motorID = 14; // TODO: Get the ID
        public static final int hallEffectChannel = 7;
        public static final double gearReduction = ((64.0 / 22.0) * 25.0) / 1.0;
        public static final double pivotKP = 0.04;
        public static final double threshAngle = 0.05; // deg
        public static final int towerCurrentLimit = 20;
        public static final double maxNegPower = -0.2;
        public static final double maxPosPower = 0.2;
        public static final int minAngle = 0;
        public static final int maxAngle = 50; //TODO: GET A CORRECT VALUE

    }

    public static final class Climber {
        public static final int leftMotorID = 11; // TODO: Get the ID
        public static final int rightMotorID = 13; // TODO: Get the ID
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
                        Units.inchesToMeters(-11),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(0)),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-16),
                    Units.degreesToRadians(180)
                ));
    }

    public static final class Logging {
        public static ShuffleboardTab driveTab = Shuffleboard.getTab("Drive"); // Turn Angles, Gyro, Etc.
        public static ShuffleboardTab intakeShooterTowerTab = Shuffleboard.getTab("Intake / Shooter");
        public static ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
        public static ShuffleboardTab poseEstimationTab = Shuffleboard.getTab("Odometry");
        public static ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");

    }
    public static final class Auto {
        public static final double towerAngleInitial = 255.0;
        
    }
}