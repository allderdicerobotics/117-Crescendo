package frc.robot.misc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.ThriftyEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;

public class SwerveModule {
    private final String name;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final ThriftyEncoder turnAbsoluteEncoder;
    private final SparkPIDController drivePIDController;

    // private final SparkPIDController turnPIDController;
    private final PIDController turnPIDController = new PIDController(
            Constants.Swerve.angleKP,
            Constants.Swerve.angleKI,
            Constants.Swerve.angleKD);
    

    private GenericEntry driveSpeed, turnAngle, desiredSpeed, desiredAngle;

    
    public SwerveModule(int driveMotorID, int turningMotorID, ThriftyEncoder thriftyEncoder, String name,
            boolean driveInvert, boolean turnInvert) {
        this.name = name;

        /* Configure Driving Motor, Encoder, and PIDController */
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        configDriveMotor(driveInvert);

        /* Configure Turning Motor, Encoder, and PIDController */
        turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        turnAbsoluteEncoder = thriftyEncoder;
        // turnPIDController = turnMotor.getPIDController();
        configTurnMotor(turnInvert);

        driveSpeed = Constants.Logging.driveTab
            .add("Drive " + name, 0).getEntry();
        turnAngle = Constants.Logging.driveTab
            .add("Turn " + name, 0).getEntry();
        desiredSpeed = Constants.Logging.driveTab
            .add("Speed " + name, 0).getEntry();
        desiredAngle = Constants.Logging.driveTab
            .add("State " + name, 0).getEntry();

    }

    private void configDriveMotor(boolean driveInvert) {

        driveMotor.restoreFactoryDefaults();

        driveMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus0,
            500
        );

        driveMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus1,
            500
        );

        driveMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2,
            500
        );

        driveMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus3,
            0
        );

        driveMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus4,
            0
        );

        driveMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus5,
            0
        );
        driveMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus0,
            0
        );

        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(driveInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveIdleMode);

        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        /* Setup Driving PID */
        drivePIDController.setP(Constants.Swerve.driveKP, 0);
        drivePIDController.setI(Constants.Swerve.driveKI, 0);
        drivePIDController.setD(Constants.Swerve.driveKD, 0);
        drivePIDController.setFF(Constants.Swerve.driveKFF, 0);

        drivePIDController.setOutputRange(-1, 1);
        
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    private void configTurnMotor(boolean turnInvert) {

        turnMotor.restoreFactoryDefaults();

        turnMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus0,
            500
        );

        turnMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus1,
            500
        );

        turnMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2,
            500
        );

        turnMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus3,
            0
        );

        turnMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus4,
            0
        );

        turnMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus5,
            0
        );
        turnMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus0,
            0
        );
        
        turnMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        turnMotor.setInverted(turnInvert);
        turnMotor.setIdleMode(Constants.Swerve.angleIdleMode);

        /*
         * Limit the PID Controller's input range between -pi and pi and set the input
         * to be continuous.
         */
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        turnPIDController.reset();
        // turnPIDController.setP(Constants.Swerve.angleKP);
        // turnPIDController.setI(Constants.Swerve.angleKI);
        // turnPIDController.setD(Constants.Swerve.angleKD);
        // turnPIDController.setFF(Constants.Swerve.angleKFF);
        // turnPIDController.setOutputRange(-1,1);
        // turnPIDController.setPositionPIDWrappingEnabled(true);
        // turnPIDController.setPositionPIDWrappingMinInput(0);
        // turnPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);
        
        
        turnMotor.burnFlash();

    }

    public void setModuleIdleMode(IdleMode driveIdleMode, IdleMode turnIdleMode) {
        turnMotor.setIdleMode(turnIdleMode);
        driveMotor.setIdleMode(driveIdleMode);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {

        /* Optimize the reference state to avoid spinning further than 90 degrees */
        desiredState = SwerveModuleState.optimize(desiredState, turnAbsoluteEncoder.get());
        setAngle(desiredState);
        setSpeed(desiredState, openLoop);

        driveSpeed.setDouble(driveEncoder.getVelocity());
        turnAngle.setDouble(turnAbsoluteEncoder.get().getDegrees());
        desiredSpeed.setDouble(desiredState.speedMetersPerSecond);
        desiredAngle.setDouble(desiredState.angle.getDegrees());

    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        if (Math.abs(desiredState.angle.getRadians() - turnAbsoluteEncoder.get().getRadians()) < Units.degreesToRadians(0.1)) {
            turnMotor.set(0);
        } else {
            final double turnOutput = turnPIDController.calculate(turnAbsoluteEncoder.get().getRadians(),
                    desiredState.angle.getRadians());
            turnMotor.setVoltage(turnOutput);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean openLoop) {
        /* Using only closed loop (PID) control from now on */
        if (openLoop) {
            double output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(output);
        } else {
            drivePIDController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity, // units for setpoint (rpm)
                    0// units for feedforward,
            );
        }
    }

    public void reset() {
        driveEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        /* Convert Encoder Readings (Rotations) to SwerveModulePosition's Meters field */
        double distanceMeters = driveEncoder.getPosition();
        return new SwerveModulePosition(distanceMeters, turnAbsoluteEncoder.get());
    }

    public SwerveModuleState getState() {
        /* Convert Encoder Readings (RPM) to SwerveModuleState's m/s field*/
        return new SwerveModuleState(driveEncoder.getVelocity(), turnAbsoluteEncoder.get());
    }
}
