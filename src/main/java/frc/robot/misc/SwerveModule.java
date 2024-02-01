package frc.robot.misc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.ThriftyEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.kinematics.*;

public class SwerveModule {
    private final String name;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;


    private final RelativeEncoder driveEncoder;
    private final ThriftyEncoder turnAbsoluteEncoder;
    private final SparkPIDController drivePIDController;
    private final PIDController turnPIDController = new PIDController(
        Constants.Swerve.angleKP,
        Constants.Swerve.angleKI,
        Constants.Swerve.angleKD
    );
    
    
    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(
        Constants.Swerve.angleKS,
        Constants.Swerve.angleKV
    );
    
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
        turnAbsoluteEncoder = thriftyEncoder;
        configTurnMotor(turnInvert);

    }

    private void configDriveMotor(boolean driveInvert){

        driveMotor.restoreFactoryDefaults();
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

        /*  Limit the PID Controller's input range between -pi and pi and set the input
        to be continuous. */
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        turnPIDController.reset();
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
        

        SmartDashboard.putNumber("Drive " + name, driveEncoder.getPosition());
        SmartDashboard.putNumber("Turn " + name,turnAbsoluteEncoder.get().getDegrees());
        SmartDashboard.putNumber("State " + name, desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Speed " + name, desiredState.speedMetersPerSecond);
        
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        if (Math.abs(desiredState.angle.getRadians() - turnAbsoluteEncoder.get().getRadians()) < 0.01){
            turnMotor.set(0);
        }else {
            final double turnOutput = turnPIDController.calculate(turnAbsoluteEncoder.get().getRadians(),
                desiredState.angle.getRadians());
            turnMotor.set(turnOutput/Constants.Swerve.motorVoltage);
        }
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
    }

    public SwerveModulePosition getPosition() {
        /* Convert Encoder Readings (RPM) to SwerveModulePosition's Meters field */
        double distanceMeters = driveEncoder.getPosition() * Constants.Swerve.driveConversionPositionFactor; 
        return new SwerveModulePosition(distanceMeters,turnAbsoluteEncoder.get());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), turnAbsoluteEncoder.get());
    }
}
