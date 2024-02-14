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
    private CANSparkMax topMotor, bottomMotor;
    private RelativeEncoder topEncoder;
    private SparkPIDController shooterPID;

    public Shooter() {
        topMotor = new CANSparkMax(Constants.Shooter.topID, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(Constants.Shooter.bottomID, MotorType.kBrushless);

        topEncoder = topMotor.getEncoder(); // Only one encoder needed for velocity calc

        shooterPID = topMotor.getPIDController(); // Only one pid controller needed because of leader-follower

        configShooter();

    }

    public void run(double rpm) {

        // topMotor.set(1);
        System.out.println(topEncoder.getVelocity());
        shooterPID.setReference(rpm,
                ControlType.kVelocity,
        0);
    }

    public boolean atSpeed(double rpm) {
        return Math.abs(topEncoder.getVelocity() - rpm) < Constants.Shooter.threshRPM;
    }

    public void stop() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    private void configShooter() {

        
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);
        bottomMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);

        topMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);
        bottomMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);

        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);

        topMotor.setInverted(true);
        bottomMotor.follow(topMotor, true); // Other Motor will be moving in opposite direction

        topMotor.burnFlash();
        bottomMotor.burnFlash();

        shooterPID.setP(Constants.Shooter.shooterKP);
        shooterPID.setI(Constants.Shooter.shooterKI);
        shooterPID.setD(Constants.Shooter.shooterKD);
        shooterPID.setFF(Constants.Shooter.shooterKFF);

        topEncoder.setVelocityConversionFactor(1.0); // Velocity = RPM

    }

}
