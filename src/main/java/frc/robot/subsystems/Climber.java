package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Climber extends SubsystemBase {
    private final CANSparkMax climberMotor;

    private final SparkPIDController climberPIDController;

    private final RelativeEncoder climberEncoder;

    public Climber(int motorID, boolean inverted) {
        /* Add Configuration Settings Here (inversions, outputrange, etc.) */
        climberMotor = new CANSparkMax(motorID, MotorType.kBrushless);

        climberEncoder = climberMotor.getEncoder();

        climberPIDController = climberMotor.getPIDController();

        configClimber(inverted);
    }

    public void moveUp() {
        if (withinLegalBounds()){
            climberMotor.set(0.5);
        }
    }
    public void moveDown(){
        if (withinLegalBounds()){
            climberMotor.set(-0.5);
        }
    }

    
    public double getPosition() {
        return climberEncoder.getPosition();
    }

    public void runZero() {
        climberMotor.set(Constants.Climber.homingSpeed);
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public void zero() {
        climberEncoder.setPosition(0);
    }


    public boolean isClimberZeroed() {
        return (climberMotor.getOutputCurrent() < Constants.Climber.homeCurrent);
    }

    private boolean withinLegalBounds(){
        var currentPosition = getPosition();
        return (0 <  currentPosition && currentPosition < Constants.Climber.legalMax);
    }
    private void configClimber(boolean inverted) {
        climberMotor.restoreFactoryDefaults();

        climberMotor.setSmartCurrentLimit(Constants.Climber.currentLimit);

        climberMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);

        climberMotor.setIdleMode(IdleMode.kBrake);

        climberMotor.setInverted(inverted);

        climberPIDController.setFeedbackDevice(climberEncoder);

        climberEncoder.setPositionConversionFactor(Constants.Climber.positionConversionFactor);
        climberEncoder.setVelocityConversionFactor(Constants.Climber.velocityConversionFactor);

        climberPIDController.setP(Constants.Climber.climberKP);

        climberPIDController.setI(Constants.Climber.climberKI);

        climberPIDController.setD(Constants.Climber.climberKD);

        climberPIDController.setOutputRange(-0.5, 0.5);

        climberMotor.burnFlash();
    }

}
