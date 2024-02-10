package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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
    private static RelativeEncoder hoodEncoder, leftShootEncoder;
    private static SparkPIDController hoodPIDController;
    private static PIDController shooterPID;
    private static SimpleMotorFeedforward shooterFF;
    private static double desiredAngle;
     
    public Shooter(){
        leftShootMotor = new CANSparkMax(Constants.Shooter.leftID, MotorType.kBrushless);
        rightShootMotor = new CANSparkMax(Constants.Shooter.rightID, MotorType.kBrushless);
        hoodMotor = new CANSparkMax(Constants.Shooter.hoodID, MotorType.kBrushless);
        
        leftShootEncoder = leftShootMotor.getEncoder();
        hoodEncoder = hoodMotor.getEncoder();
        hoodPIDController = hoodMotor.getPIDController();
        
        desiredAngle = 0;

        shooterPID = new PIDController(
            Constants.Shooter.shooterKP,
            Constants.Shooter.shooterKI,
            Constants.Shooter.shooterKD
        );

        shooterFF = new SimpleMotorFeedforward(
            Constants.Shooter.shooterKS,
            Constants.Shooter.shooterKFF  
        );

        configShooter(true);
        shootAngleTable = new InterpolatingDoubleTreeMap();
        populateShooterTable();


    }

    public void setAngleShooter(double shooterAngle){
        setDesiredShooterAngle(shooterAngle);
        hoodPIDController.setReference(
            shooterAngle, 
            ControlType.kPosition
        );
    }

    public boolean atAngle(){
        return Math.abs(getShooterAngle() - desiredAngle) <= Constants.Shooter.threshAngle;
    }
    public void shoot(double rpm){
        double pidOutput = shooterPID.calculate(leftShootEncoder.getVelocity(),rpm);
        double ffOutput = shooterFF.calculate(rpm);
        double output = Math.max(pidOutput + ffOutput,Constants.Shooter.maxNegPower);
        
        leftShootMotor.set(output);
    }

    public void stop(){
        leftShootMotor.stopMotor();
        rightShootMotor.stopMotor();
    }
    public double getShooterAngle(){
        return hoodEncoder.getPosition();
    }
    public void setDesiredShooterAngle(double shooterAngle){
        desiredAngle = shooterAngle;
    }
    public double interpolateAngle(double distance){
        return shootAngleTable.get(distance);
    }
    private void populateShooterTable(){
        // Fill with the tested values (limelight-based distance, arm angle)
        shootAngleTable.put(0.0,0.0);
    }
    private void configShooter(boolean shooterInverted){

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
