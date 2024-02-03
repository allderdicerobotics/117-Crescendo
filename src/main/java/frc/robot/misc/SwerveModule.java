package frc.robot.misc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.ThriftyEncoder;

import com.gos.lib.swerve.SwerveDrivePublisher;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
    private final String name;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;


    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final ThriftyEncoder turnAbsoluteEncoder;
    private final SparkPIDController drivePIDController;
    private final SparkPIDController turnPIDController;

    
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
        Constants.Swerve.driveKS,
        Constants.Swerve.driveKV,
        Constants.Swerve.driveKA
    );
    public SwerveModule(int driveMotorID, int turningMotorID, ThriftyEncoder thriftyEncoder, String name, boolean driveInvert, boolean turnInvert) {
        this.name = name;

        /* Configure Driving Motor, Encoder, and PIDController */
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        configDriveMotor(driveInvert);

        /* Configure Turning Motor, Encoder, and PIDController */
        turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        turnRelativeEncoder = turnMotor.getEncoder();
        turnAbsoluteEncoder = thriftyEncoder;
        turnPIDController = turnMotor.getPIDController();
        configTurnMotor(turnInvert);

        reset();

    }

    private void configDriveMotor(boolean driveInvert){

        driveMotor.restoreFactoryDefaults();
        drivePIDController.setFeedbackDevice(driveEncoder);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(driveInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveIdleMode);
                
        /* Setup Driving PID */
        drivePIDController.setP(Constants.Swerve.driveKP, 0);
        drivePIDController.setI(Constants.Swerve.driveKI, 0);
        drivePIDController.setD(Constants.Swerve.driveKD, 0);
        drivePIDController.setOutputRange(-1, 1);
        drivePIDController.setSmartMotionMaxVelocity(10000, 0);
        drivePIDController.setFF(Constants.Swerve.driveKFF, 0);

        drivePIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        drivePIDController.setSmartMotionMaxAccel(Constants.Swerve.maxAccel, 0);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }
    private void configTurnMotor(boolean turnInvert){

        turnMotor.restoreFactoryDefaults();
        turnMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        turnMotor.setInverted(turnInvert);
        turnMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        
        turnRelativeEncoder.setPositionConversionFactor(Constants.Swerve.turnConversionPositionFactor);
        turnRelativeEncoder.setVelocityConversionFactor(Constants.Swerve.turnConversionVelocityFactor);

        /*  Limit the PID Controller's input range between -pi and pi and set the input
        to be continuous. */
        turnPIDController.setFeedbackDevice(turnRelativeEncoder);

        turnPIDController.setP(Constants.Swerve.angleKP);
        turnPIDController.setI(Constants.Swerve.angleKI);
        turnPIDController.setD(Constants.Swerve.angleKD);


        turnPIDController.setPositionPIDWrappingEnabled(true);
        turnPIDController.setPositionPIDWrappingMinInput(Constants.Swerve.turnPIDMinInput);
        turnPIDController.setPositionPIDWrappingMaxInput(Constants.Swerve.turnPIDMaxInput);

        
        turnMotor.burnFlash();
    }

    public void setModuleIdleMode(IdleMode driveIdleMode, IdleMode turnIdleMode) {
        turnMotor.setIdleMode(turnIdleMode);
        driveMotor.setIdleMode(driveIdleMode);
    }
    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {

        /* Optimize the reference state to avoid spinning further than 90 degrees */
        // desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRelativeEncoder.getPosition()));
        // desiredState = nonContinuousOptimize(desiredState, new Rotation2d(turnRelativeEncoder.getPosition()));
        setAngle(desiredState);
        setSpeed(desiredState, openLoop);
        
        
        SmartDashboard.putNumber("Drive " + name, driveEncoder.getPosition());
        SmartDashboard.putNumber("Turn " + name,turnAbsoluteEncoder.get().getDegrees());
        SmartDashboard.putNumber("TurnRel " + name, new Rotation2d(turnRelativeEncoder.getPosition()).getDegrees());
        SmartDashboard.putNumber("State " + name, desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Speed " + name, desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("angle diff " + name, (turnRelativeEncoder.getPosition() - turnAbsoluteEncoder.get().getRadians()) / (2 * Math.PI) * 360);
        // SmartDashboard.putNumber(""), 0)
        
    }

    private void setAngle(SwerveModuleState desiredState) {
        turnPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        // turnPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        // if (Math.abs(desiredState.angle.getRadians() - turnAbsoluteEncoder.get().getRadians()) < 0.01){
        //     turnMotor.set(0);
        // }else {
            // final double turnOutput = turnPIDController.calculate(turnAbsoluteEncoder.get().getRadians(),
            //     desiredState.angle.getRadians());
            // final double turnFeedforwardOut = turnFF.calculate(turnPIDController.getSetpoint().velocity);
            // turnMotor.setVoltage(turnOutput + turnFeedforwardOut);
        // }
    }
    private void setSpeed(SwerveModuleState desiredState, boolean openLoop){
        if (openLoop){
            double output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(output);
        } else{
            drivePIDController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity, // units for setpoint (rpm)
                0,// units for feedforward,
                driveFeedForward.calculate(desiredState.speedMetersPerSecond)
            );
        }
    }
    public void reset() {
        driveEncoder.setPosition(0);
        turnMotor.set(0);
        REVLibError err = turnRelativeEncoder.setPosition(turnAbsoluteEncoder.get().getRadians());
        
        if (err != REVLibError.kOk) {
            System.out.println("Didnt work");
        }
    }

    public SwerveModulePosition getPosition() {
        /* Convert Encoder Readings (RPM) to SwerveModulePosition's Meters field */
        double distanceMeters = driveEncoder.getPosition() * Constants.Swerve.driveConversionPositionFactor; 
        return new SwerveModulePosition(distanceMeters,turnAbsoluteEncoder.get());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnRelativeEncoder.getPosition()));
    }
    
    private SwerveModuleState nonContinuousOptimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double targetAngle = placeInAppropriateScope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
    
    private double placeInAppropriateScope(double scopeRef, double newAngle){
        double lowerBound, upperBound, lowerOffset;
        lowerOffset = scopeRef % 360;

        if (lowerOffset >= 0){
            lowerBound = scopeRef - lowerOffset;
            upperBound = scopeRef - (360 - lowerOffset);
        } else{
            upperBound = scopeRef - lowerOffset;
            lowerBound = scopeRef - (360 + lowerOffset);
        }

        while (newAngle < lowerBound){
            newAngle += 360;
        }
        while (newAngle > upperBound){
            newAngle -= 360;
        }
        if (newAngle - scopeRef > 180){
            newAngle -= 360;
        } else if (newAngle - scopeRef < -180){
            newAngle += 360;
        }
        return newAngle;
    } 
}
