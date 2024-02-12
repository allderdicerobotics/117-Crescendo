package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkMax leftShootMotor, rightShootMotor;
    private RelativeEncoder leftShootEncoder;
    private SparkPIDController shooterPID;

    public Shooter() {
        leftShootMotor = new CANSparkMax(Constants.Shooter.leftID, MotorType.kBrushless);
        rightShootMotor = new CANSparkMax(Constants.Shooter.rightID, MotorType.kBrushless);

        leftShootEncoder = leftShootMotor.getEncoder(); // Only one encoder needed for velocity calc

        shooterPID = leftShootMotor.getPIDController(); // Only one pid controller needed because of leader-follower

        configShooter();

    }

    public void run(double rpm) {
        shooterPID.setReference(rpm,
                ControlType.kVelocity,
                0);
    }

    public boolean nearSpeed(double rpm) {
        return Math.abs(leftShootEncoder.getVelocity() - rpm) < Constants.Shooter.threshRPM;
    }

    public void stop() {
        leftShootMotor.stopMotor();
        rightShootMotor.stopMotor();
    }

    private void configShooter() {

        shooterPID.setP(Constants.Shooter.shooterKP);
        shooterPID.setI(Constants.Shooter.shooterKI);
        shooterPID.setD(Constants.Shooter.shooterKD);
        shooterPID.setFF(Constants.Shooter.shooterKFF);

        leftShootMotor.restoreFactoryDefaults();
        rightShootMotor.restoreFactoryDefaults();

        leftShootMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);
        rightShootMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);

        leftShootMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);
        rightShootMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);

        leftShootMotor.setIdleMode(IdleMode.kCoast);
        rightShootMotor.setIdleMode(IdleMode.kCoast);

        leftShootMotor.setInverted(false);
        rightShootMotor.follow(leftShootMotor, true); // Other Motor will be moving in opposite direction

        leftShootMotor.burnFlash();
        rightShootMotor.burnFlash();

        leftShootEncoder.setVelocityConversionFactor(1.0); // Velocity = RPM

    }

}
