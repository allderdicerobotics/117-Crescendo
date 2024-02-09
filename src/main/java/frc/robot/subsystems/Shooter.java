package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Shooter extends SubsystemBase{
    private static InterpolatingDoubleTreeMap shootAngleTable;
    private static CANSparkMax hoodMotor, leftShootMotor, rightShootMotor;
    private static RelativeEncoder hoodEncoder, leftShootEncoder, rightShootEncoder;
    private static SparkPIDController hoodPIDController;
    private static PIDController shooterPID;
    private static SimpleMotorFeedforward shooterFF;
     
    public Shooter(){
        leftShootMotor = new CANSparkMax(Constants.Shooter.leftID, MotorType.kBrushless);
        rightShootMotor = new CANSparkMax(Constants.Shooter.rightID, MotorType.kBrushless);
        hoodMotor = new CANSparkMax(Constants.Shooter.hoodID, MotorType.kBrushless);
        
        leftShootEncoder = leftShootMotor.getEncoder();
        rightShootEncoder = rightShootMotor.getEncoder();
        hoodEncoder = hoodMotor.getEncoder();
        hoodPIDController = hoodMotor.getPIDController();

        configShooter(true);
        shootAngleTable = new InterpolatingDoubleTreeMap();
        populateShooterTable();


    }

    public static void setAngleShooter(double shooterAngle){

    }

    public void shoot(double rpm){
        rpm = Math.min(rpm,Constants.Shooter.maxRPM);
        double pidOutput = shooterPID.calculate(leftShootEncoder.getVelocity(),rpm);
        double ffOutput = shooterFF.calculate(rpm);
        leftShootMotor.set(pidOutput + ffOutput);
    }
    public static double getShooterAngle(){
        return hoodEncoder.getPosition();
    }
    public static double interpolateAngle(double distance){
        return shootAngleTable.get(distance);
    }
    private void populateShooterTable(){
        // Fill with the tested values (limelight-based distance, arm angle)
        shootAngleTable.put(0.0,0.0);
    }
    private void configShooter(boolean shooterInverted){

        shooterPID = new PIDController(
            Constants.Shooter.shooterKP,
            Constants.Shooter.shooterKI,
            Constants.Shooter.shooterKD
        );

        shooterFF = new SimpleMotorFeedforward(
            Constants.Shooter.shooterKS,
            Constants.Shooter.shooterKFF  
        );


        shooterPID.setTolerance(Constants.Shooter.maxRPM/20);
        shooterPID.setIntegratorRange(-Constants.Shooter.intRangePID, Constants.Shooter.intRangePID);

        leftShootMotor.restoreFactoryDefaults();
        rightShootMotor.restoreFactoryDefaults();

        leftShootMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);
        rightShootMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);

        leftShootMotor.setIdleMode(IdleMode.kCoast);
        rightShootMotor.setIdleMode(IdleMode.kCoast);

        leftShootMotor.setInverted(false);
        rightShootMotor.follow(leftShootMotor,true);

        leftShootMotor.burnFlash();
        rightShootMotor.burnFlash();

        leftShootEncoder.setVelocityConversionFactor(1.0);
        
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setSmartCurrentLimit(Constants.Shooter.shooterCurrentLimit);
        hoodMotor.setIdleMode(IdleMode.kBrake);

        hoodEncoder.setPositionConversionFactor(Constants.Shooter.hoodReduction);
        hoodEncoder.setPosition(0);

        hoodPIDController.setP(Constants.Shooter.hoodKP);
        hoodPIDController.setOutputRange(Constants.Shooter.maxNegPower, Constants.Shooter.maxPosPower);
        
        hoodMotor.burnFlash();
        hoodMotor.setInverted(shooterInverted);
    }
    
}
