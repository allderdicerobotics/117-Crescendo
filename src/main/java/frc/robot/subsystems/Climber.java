package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Climber extends SubsystemBase{
    public double currentSetpoint;
    
    private final CANSparkMax leftClimber, rightClimber;  
    
    private final SparkPIDController leftPIDController, rightPIDController;

    private final RelativeEncoder leftEncoder, rightEncoder;

    public Climber(){
        /* Add Configuration Settings Here (inversions, outputrange, etc.) */
        leftClimber = new CANSparkMax(Constants.Climber.leftMotorID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(Constants.Climber.rightMotorID, MotorType.kBrushless);

        leftEncoder = leftClimber.getEncoder();
        rightEncoder = rightClimber.getEncoder();

        leftPIDController = leftClimber.getPIDController();
        rightPIDController = rightClimber.getPIDController();

        configClimber(false);
    }

    private void configClimber(boolean leftInverted){
        leftClimber.restoreFactoryDefaults();
        rightClimber.restoreFactoryDefaults();

        leftClimber.setSmartCurrentLimit(Constants.Climber.currentLimit);
        rightClimber.setSmartCurrentLimit(Constants.Climber.currentLimit);

        leftClimber.enableVoltageCompensation(Constants.globalVoltageCompensation);
        rightClimber.enableVoltageCompensation(Constants.globalVoltageCompensation);

        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftClimber.setInverted(leftInverted);
        rightClimber.setInverted(!leftInverted);

        leftEncoder.setPositionConversionFactor(Constants.Climber.positionConversionFactor);
        leftEncoder.setVelocityConversionFactor(Constants.Climber.velocityConversionFactor);

        rightEncoder.setPositionConversionFactor(Constants.Climber.positionConversionFactor);
        rightEncoder.setVelocityConversionFactor(Constants.Climber.velocityConversionFactor);

        leftPIDController.setP(Constants.Climber.climberKP);
        rightPIDController.setP(Constants.Climber.climberKP);

        leftPIDController.setI(Constants.Climber.climberKI);
        rightPIDController.setI(Constants.Climber.climberKI);

        leftPIDController.setD(Constants.Climber.climberKD);
        rightPIDController.setD(Constants.Climber.climberKD);

    }
    

}
