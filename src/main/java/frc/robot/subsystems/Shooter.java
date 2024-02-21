package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkMax topMotor, bottomMotor;
    private RelativeEncoder topEncoder, bottomEncoder;
    private SparkPIDController topShooterPID, bottomShooterPID;

    public Shooter() {
        topMotor = new CANSparkMax(Constants.Shooter.topID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(Constants.Shooter.bottomID, MotorType.kBrushless);

        topEncoder = topMotor.getEncoder(); // Only one encoder needed for velocity calc
        bottomEncoder = bottomMotor.getEncoder();

        topShooterPID = topMotor.getPIDController(); // Only one pid controller needed because of leader-follower
        bottomShooterPID = bottomMotor.getPIDController();

        configShooter();

    }

    public void run(double rpm) {

        // topMotor.set(1);
        // topMotor.set(1);
        // bottomMotor.set(1);
        topShooterPID.setReference(rpm,
                ControlType.kVelocity,
        0);

        bottomShooterPID.setReference(
            rpm, 
            ControlType.kVelocity,
        0);
    }

    public double topEncoderVelocity(){
        return topEncoder.getVelocity();
    }
    public double bottomEncoderVelocity(){
        return bottomEncoder.getVelocity();
    }
    public void runAmp(double rpm){
        topMotor.set(0.05);
        bottomMotor.set(0.05);
        // topShooterPID.setReference(
        //     rpm, 
        //     ControlType.kVelocity,
        //     0);

        // bottomShooterPID.setReference(
        //     rpm, 
        //     ControlType.kVelocity,
        //     0);
    }

    public boolean atSpeed(double rpm) {
        return Math.abs(topEncoder.getVelocity() - rpm) <= Constants.Shooter.threshRPM && Math.abs(bottomEncoder.getVelocity() - rpm) <= Constants.Shooter.threshRPM;
    }

    public boolean aboveSpeed(double rpm){
        return topEncoder.getVelocity() >= rpm || bottomEncoder.getVelocity() >= rpm;
    }

    public void stop() {
        topMotor.set(0);
        bottomMotor.set(0);
        // topMotor.stopMotor();
        // bottomMotor.stopMotor();
    }

    private void configShooter() {
        
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2,
            0
        );

        topMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus3,
            0
        );

        topMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus4,
            0   
        );

        topMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus5,
            0
        );

        topMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus6,
            0
        );

        bottomMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2,
            0
        );

        bottomMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus3,
            0
        );

        bottomMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus4,
            0   
        );

        bottomMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus5,
            0
        );

        bottomMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus6,
            0
        );
        
        topMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);
        bottomMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);

        topMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);
        bottomMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);

        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);

        topMotor.setInverted(true);
        bottomMotor.setInverted(false);
        // bottomMotor.follow(topMotor, true); // Other Motor will be moving in opposite direction

        topMotor.burnFlash();
        bottomMotor.burnFlash();

        topShooterPID.setP(Constants.Shooter.shooterKP);
        topShooterPID.setI(Constants.Shooter.shooterKI);
        topShooterPID.setD(Constants.Shooter.shooterKD);
        topShooterPID.setFF(Constants.Shooter.shooterKFF);

        bottomShooterPID.setP(Constants.Shooter.shooterKP);
        bottomShooterPID.setI(Constants.Shooter.shooterKI);
        bottomShooterPID.setD(Constants.Shooter.shooterKD);
        bottomShooterPID.setFF(Constants.Shooter.shooterKFF);

        topEncoder.setVelocityConversionFactor(1.0); // Velocity = RPM
        bottomEncoder.setVelocityConversionFactor(1.0);

    }

}
