package frc.robot.subsystems;

\import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Climber extends SubsystemBase{
    public double currentSetpoint;
    
    private final CANSparkMax leftClimber = new CANSparkMax(Constants.Climber.leftMotorID, MotorType.kBrushless);
    private final CANSparkMax rightClimber = new CANSparkMax(Constants.Climber.rightMotorID, MotorType.kBrushless);
    
    private final SparkPIDController leftPIDController = leftClimber.getPIDController();
    private final SparkPIDController rightPIDController = rightClimber.getPIDController();

    private final RelativeEncoder leftEncoder = leftClimber.getEncoder();
    private final RelativeEncoder rightEncoder = rightClimber.getEncoder();

    public Climber(){
        /* Add Configuration Settings Here (inversions, outputrange, etc.) */
        
    }
    
    
}
