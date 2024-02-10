package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.playingwithfusion.TimeOfFlight.Status;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Indexer extends SubsystemBase {
    private CANSparkMax indexMotor;
    private TimeOfFlight beamBreakSensor;
    
    public Indexer(){
        indexMotor = new CANSparkMax(Constants.Indexer.motorID, MotorType.kBrushless);
        beamBreakSensor = new TimeOfFlight(Constants.Indexer.sensorID);
        beamBreakSensor.setRangingMode(RangingMode.Short, 24);
        indexMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setInverted(false);
    }

    public void runIntake(){
        indexMotor.set(1.0);
    }
    public void stopIndexer(){
        indexMotor.stopMotor();
    }
    public boolean indexerFilled(){
        if (beamBreakSensor.getStatus() == Status.Valid){
            return beamBreakSensor.getRange() <= Constants.Indexer.threshDist;
        }
        return false;
    }
}
