package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.misc.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tower extends SubsystemBase {

    private InterpolatingDoubleTreeMap towerAngleTable;
    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotEncoder;
    private SparkPIDController pivotPIDController;

    public Tower() {
        pivotMotor = new CANSparkMax(Constants.Tower.motorID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = pivotMotor.getPIDController();

        configTower();

        towerAngleTable = new InterpolatingDoubleTreeMap(); // Use Linear Interpolation to Estimate Correct Angle of Tower
        populateTowerAngleTable();
    }

    public void moveTowerUp(){
        
        if (0 <= getPivotAngle()){
            pivotMotor.set(0.5);
        }
        
    }

    public void moveTowerDown(){
        if (getPivotAngle() <= Constants.Tower.maxAngle){
            pivotMotor.set(-0.5);
        }
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

    public boolean nearAngle(double angle) {
        return Math.abs(getPivotAngle() - angle) <= Constants.Tower.threshAngle;
    }

    public void stop(){
        pivotMotor.stopMotor();
    }
    private void configTower() {
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.enableVoltageCompensation(Constants.globalVoltageCompensation);
        pivotMotor.setSmartCurrentLimit(Constants.Tower.towerCurrentLimit);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder.setPositionConversionFactor(Constants.Tower.gearReduction);
        pivotEncoder.setPosition(0);

        pivotPIDController.setP(Constants.Tower.pivotKP);
        pivotPIDController.setOutputRange(Constants.Tower.maxNegPower, Constants.Tower.maxPosPower);

        pivotMotor.burnFlash();
        pivotMotor.setInverted(true);
    }
}
