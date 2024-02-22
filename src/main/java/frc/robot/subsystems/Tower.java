package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.misc.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tower extends SubsystemBase {

    private InterpolatingDoubleTreeMap towerAngleTable;
    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotEncoder;
    private SparkPIDController pivotPIDController;
    private double currentSetpoint = 0;

    public Tower() {
        pivotMotor = new CANSparkMax(Constants.Tower.motorID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = pivotMotor.getPIDController();

        configTower();

        towerAngleTable = new InterpolatingDoubleTreeMap(); // Use Linear Interpolation to Estimate Correct Angle of Tower
        populateTowerAngleTable();

        Constants.Logging.intakeShooterTowerTab.addDouble("Tower Angle", this::getPivotAngle).withSize(2,1);
        
    }

    public void moveTower(double speed){
        pivotMotor.set(speed);        
    }

    public void setPivotAngle(double pivotAngle) {
        pivotPIDController.setReference(
                pivotAngle,
                ControlType.kPosition);
    }

    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    public double interpolateAngle(double distance) {
        return towerAngleTable.get(distance);
    }

    private void populateTowerAngleTable() {
        // Fill with the tested values (vision-based distance, tower angle)
        towerAngleTable.put(0.0, 0.0);
    }

    public void zero(){
        this.currentSetpoint = 0;
        pivotEncoder.setPosition(0);

    }

    public boolean nearAngle(double angle) {
        return Math.abs(getPivotAngle() - angle) <= Constants.Tower.threshAngle;
    }

    public void stop(){
        pivotMotor.stopMotor();
    }

    private void configTower() {

        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus1, 
            500);

        pivotMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2, 
            500);

        pivotMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus3,
            0
        );

        pivotMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus4,
            0   
        );

        pivotMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus5,
            0
        );

        pivotMotor.setPeriodicFramePeriod(
            PeriodicFrame.kStatus6,
            0
        );
        pivotMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);
        pivotMotor.setSmartCurrentLimit(Constants.Tower.towerCurrentLimit);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder.setPositionConversionFactor(Constants.Tower.gearReduction);

        pivotPIDController.setP(Constants.Tower.pivotKP);
        pivotPIDController.setOutputRange(Constants.Tower.maxNegPower, Constants.Tower.maxPosPower);
        pivotMotor.setInverted(true);
        
        pivotMotor.burnFlash();
        
    }
}
