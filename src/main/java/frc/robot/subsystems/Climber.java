package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Climber extends SubsystemBase {
    public double currentSetpoint;
    public boolean zeroed = false;

    private final CANSparkMax climberMotor;

    private final SparkPIDController climberPIDController;

    private final RelativeEncoder climberEncoder;

    public Climber(int motorID) {
        /* Add Configuration Settings Here (inversions, outputrange, etc.) */
        climberMotor = new CANSparkMax(motorID, MotorType.kBrushless);

        climberEncoder = climberMotor.getEncoder();

        climberPIDController = climberMotor.getPIDController();

        configClimber(false);
    }

    public void setPosition(double desiredPosition){
        this.currentSetpoint = desiredPosition;
        climberPIDController.setReference(
            desiredPosition,
            ControlType.kPosition
        );
    }

    public void motionToZero() {
        climberMotor.set(Constants.Climber.homingSpeed);
    }

    public void stopClimber() {
        climberMotor.stopMotor();
    }

    public void zeroClimber() {
        climberEncoder.setPosition(0);
    }

    public boolean isClimberZeroed() {
        return (climberMotor.getOutputCurrent() < Constants.Climber.homeCurrent);
    }

    private void configClimber(boolean inverted) {
        climberMotor.restoreFactoryDefaults();

        climberMotor.setSmartCurrentLimit(Constants.Climber.currentLimit);

        climberMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);

        climberMotor.setIdleMode(IdleMode.kBrake);

        climberMotor.setInverted(inverted);

        climberEncoder.setPositionConversionFactor(Constants.Climber.positionConversionFactor);
        climberEncoder.setVelocityConversionFactor(Constants.Climber.velocityConversionFactor);

        climberPIDController.setP(Constants.Climber.climberKP);

        climberPIDController.setI(Constants.Climber.climberKI);

        climberPIDController.setD(Constants.Climber.climberKD);

        climberPIDController.setOutputRange(-0.5, 0.5);

        climberMotor.burnFlash();
    }

}
